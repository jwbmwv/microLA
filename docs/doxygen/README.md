# MicroLA Documentation Generation

This directory will contain Doxygen-generated HTML documentation.

## Generating Documentation

### Prerequisites

Install Doxygen:

**Ubuntu/Debian:**
```bash
sudo apt install doxygen graphviz
```

**macOS:**
```bash
brew install doxygen graphviz
```

**Windows:**
Download from https://www.doxygen.nl/download.html

### Build Documentation

```bash
# Option 1: Using CMake
mkdir build && cd build
cmake .. -DMICROLA_LINEAR_BUILD_DOCS=ON
make docs

# Option 2: Direct doxygen
cd microla
doxygen Doxyfile
```

### View Documentation

Open `docs/doxygen/html/index.html` in your web browser:

```bash
# Linux
xdg-open docs/doxygen/html/index.html

# macOS
open docs/doxygen/html/index.html

# Windows
start docs/doxygen/html/index.html
```

## Documentation Structure

The generated documentation includes:

- **Classes** - Full API reference for Vec, Mat, SquareMat, Quaternion
- **Namespaces** - microla and microla::constants
- **Files** - All header files with detailed descriptions
- **Examples** - Embedded code examples from comments
- **Diagrams** - Class hierarchies and include graphs (requires Graphviz)

## Customization

Edit the `Doxyfile` in the project root to customize:

- Output format (HTML, LaTeX, XML)
- Theme and colors
- What to document (private members, internals, etc.)
- Diagram generation options
- Search functionality

## CI/CD Integration

The documentation can be automatically built and deployed:

```yaml
# Example GitHub Actions workflow
- name: Build Documentation
  run: |
    sudo apt install doxygen graphviz
    doxygen Doxyfile
    
- name: Deploy to GitHub Pages
  uses: peaceiris/actions-gh-pages@v3
  with:
    github_token: ${{ secrets.GITHUB_TOKEN }}
    publish_dir: ./docs/doxygen/html
```

## Online Documentation

Pre-built documentation available at:
- https://yourusername.github.io/microla/ (when deployed)

## Troubleshooting

**Problem**: "Doxygen not found"
```bash
# Verify installation
doxygen --version

# Check PATH
which doxygen  # Linux/macOS
where doxygen  # Windows
```

**Problem**: Graphs not generated
```bash
# Install graphviz
sudo apt install graphviz  # Ubuntu
brew install graphviz      # macOS

# Verify
dot -V
```

**Problem**: Warnings about undocumented items
```
# In Doxyfile, set:
WARN_IF_UNDOCUMENTED = NO
```
