#!/usr/bin/env python3
"""Download a benchmark baseline artifact from the latest successful CI run."""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
from pathlib import Path
from typing import Optional, Tuple
import urllib.error
import urllib.parse
import urllib.request
import zipfile


API_VERSION = "2022-11-28"
USER_AGENT = "microla-benchmark-baseline-fetcher"
REDIRECT_STATUS_CODES = {301, 302, 303, 307, 308}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workflow", required=True, help="Workflow file name, for example ci.yml")
    parser.add_argument("--artifact", required=True, help="Artifact name to download")
    parser.add_argument("--branch", default="main", help="Branch to search for successful workflow runs")
    parser.add_argument("--output-dir", required=True, help="Directory where the artifact archive should be extracted")
    parser.add_argument(
        "--exclude-run-id",
        default=os.environ.get("GITHUB_RUN_ID", ""),
        help="Workflow run id to exclude from the search",
    )
    return parser.parse_args()


def build_request(url: str, token: str) -> urllib.request.Request:
    return urllib.request.Request(
        url,
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "X-GitHub-Api-Version": API_VERSION,
            "User-Agent": USER_AGENT,
        },
    )


def get_json(url: str, token: str) -> dict:
    with urllib.request.urlopen(build_request(url, token), timeout=30) as response:
        return json.load(response)


class NoRedirectHandler(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):  # type: ignore[override]
        return None


def get_bytes(url: str, token: str) -> bytes:
    opener = urllib.request.build_opener(NoRedirectHandler)
    try:
        with opener.open(build_request(url, token), timeout=60) as response:
            return response.read()
    except urllib.error.HTTPError as error:
        if error.code not in REDIRECT_STATUS_CODES:
            raise

        redirect_url = error.headers.get("Location")
        if not redirect_url:
            raise

    redirect_request = urllib.request.Request(redirect_url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(redirect_request, timeout=60) as response:
        return response.read()


def find_artifact_download_url(api_url: str, repo: str, workflow: str, artifact_name: str, branch: str,
                               exclude_run_id: str, token: str) -> Tuple[Optional[str], Optional[int]]:
    params = urllib.parse.urlencode({"branch": branch, "status": "completed", "per_page": 30})
    runs_url = f"{api_url}/repos/{repo}/actions/workflows/{workflow}/runs?{params}"
    runs_payload = get_json(runs_url, token)

    for run in runs_payload.get("workflow_runs", []):
        run_id = str(run.get("id", ""))
        if not run_id or run_id == exclude_run_id:
            continue
        if run.get("conclusion") != "success":
            continue

        artifacts_url = f"{api_url}/repos/{repo}/actions/runs/{run_id}/artifacts?per_page=100"
        artifacts_payload = get_json(artifacts_url, token)
        for artifact in artifacts_payload.get("artifacts", []):
            if artifact.get("name") != artifact_name:
                continue
            if artifact.get("expired"):
                continue
            archive_url = artifact.get("archive_download_url")
            artifact_id = artifact.get("id")
            if archive_url and artifact_id is not None:
                return archive_url, int(artifact_id)

    return None, None


def extract_archive(archive_bytes: bytes, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(archive_bytes)) as archive:
        archive.extractall(output_dir)


def main() -> int:
    args = parse_args()
    token = os.environ.get("GITHUB_TOKEN")
    api_url = os.environ.get("GITHUB_API_URL", "https://api.github.com")
    repo = os.environ.get("GITHUB_REPOSITORY")

    if not token or not repo:
        print("GitHub token or repository context missing; skipping baseline download.")
        return 0

    try:
        archive_url, artifact_id = find_artifact_download_url(
            api_url=api_url,
            repo=repo,
            workflow=args.workflow,
            artifact_name=args.artifact,
            branch=args.branch,
            exclude_run_id=args.exclude_run_id,
            token=token,
        )
    except urllib.error.HTTPError as error:
        print(f"Unable to query workflow runs for baseline artifact: HTTP {error.code}")
        return 0
    except urllib.error.URLError as error:
        print(f"Unable to query workflow runs for baseline artifact: {error.reason}")
        return 0

    if not archive_url or artifact_id is None:
        print(f"No non-expired '{args.artifact}' artifact found on successful '{args.branch}' runs.")
        return 0

    try:
        archive_bytes = get_bytes(archive_url, token)
    except urllib.error.HTTPError as error:
        print(f"Unable to download artifact {artifact_id}: HTTP {error.code}")
        return 0
    except urllib.error.URLError as error:
        print(f"Unable to download artifact {artifact_id}: {error.reason}")
        return 0

    output_dir = Path(args.output_dir)
    extract_archive(archive_bytes, output_dir)
    print(f"Downloaded '{args.artifact}' artifact {artifact_id} into {output_dir}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
