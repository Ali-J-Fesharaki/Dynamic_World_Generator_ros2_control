#!/bin/bash
# setup_env.sh: Sets up the optimized architecture within a virtual environment.
set -e

PROJECT_ROOT="/home/ajf/Dynamic_World_Generator_ros2_control"
cd "$PROJECT_ROOT"

echo "=== 1. Creating Python Virtual Environment ==="
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
    echo "Created virtual environment in .venv"
else
    echo "Virtual environment .venv already exists"
fi

# Activate virtualenv
source .venv/bin/activate

echo "=== 2. Upgrading Pip and Installing Nodeenv ==="
pip install --upgrade pip
pip install nodeenv

echo "=== 3. Integrating Node.js/NPM into Virtualenv ==="
# Check if node is already installed in virtualenv
if [ ! -f ".venv/bin/node" ]; then
    nodeenv -p --node=lts
    echo "Integrated Node.js LTS into virtualenv"
else
    echo "Node.js is already integrated in virtualenv"
fi

# Re-source activate to ensure all node paths are correct
source .venv/bin/activate

echo "=== 4. Checking Python/Node/NPM Versions ==="
python3 --version
node -v
npm -v

echo "=== 5. Installing Graphify ==="
pip install graphifyy

echo "=== 6. Installing & Configuring CodeGraph ==="
npm install -g @colbymchenry/codegraph
codegraph install --yes
codegraph init -i || echo "CodeGraph already initialized or ran with status code $?"

echo "=== 7. Installing RTK (Rust Token Killer) ==="
# RTK installs to ~/.local/bin by default, but we can set RTK_INSTALL_DIR to .venv/bin
export RTK_INSTALL_DIR="$PROJECT_ROOT/.venv/bin"
curl -fsSL https://raw.githubusercontent.com/rtk-ai/rtk/master/install.sh | sh

echo "=== 8. Installing & Configuring Repomix ==="
npm install -g repomix
if [ ! -f "repomix.config.json" ]; then
    echo "Initializing Repomix configuration..."
    # repomix --init is interactive; we can pipe yes/empty to accept defaults
    echo "" | repomix --init || true
else
    echo "Repomix configuration already exists"
fi

echo "=== 9. Installing Caveman ==="
npm install -g @juliusbrussee/caveman-code

echo "=== 10. Installing CRUX-Compress ==="
curl -fsSL https://raw.githubusercontent.com/zotoio/CRUX-Compress/main/install.py | python3 - -y

echo "=== 11. Creating Project Files ==="
# Create empty files in project root if they do not exist
touch MEMORY.md CHANGELOG.md

# Create or overwrite CLAUDE.md with CRUX frontmatter
cat << 'EOF' > CLAUDE.md
---
crux: true
---

# Project Rules

## Build Commands
- Run your build using standard project build commands.

## Code Style Guidelines
- Maintain documentation integrity and style.
EOF

echo "=== 12. Appending .venv to .gitignore ==="
if ! grep -q "^\.venv/" .gitignore; then
    echo "" >> .gitignore
    echo "# Python virtual environment" >> .gitignore
    echo ".venv/" >> .gitignore
    echo "Appended .venv/ to .gitignore"
else
    echo ".venv/ is already in .gitignore"
fi

echo "=== Setup complete! ==="
