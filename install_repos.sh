#!/bin/bash

# Usage:
#   ./install_repos.sh repos.txt /path/to/install/dir [optional:/path/to/venvs] [--setup-cfg]

REPO_LIST="$1"
INSTALL_DIR=".."
VENV_DIR=""
PATCH_CFG=false

# Process remaining arguments
shift 1
for arg in "$@"; do
    if [[ "$arg" == "--setup-cfg" ]]; then
        PATCH_CFG=true
    elif [[ -z "$VENV_DIR" ]]; then
        VENV_DIR="$arg"
    fi
done

# Default venv dir if not set
VENV_DIR="${VENV_DIR:-$HOME}"

# Colors
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
CYAN="\033[0;36m"
NC="\033[0m"

# Validate inputs
if [[ -z "$REPO_LIST" || -z "$INSTALL_DIR" ]]; then
    echo -e "${RED}❌ Usage: $0 <repo_list.txt> <install_dir> [venv_dir] [--setup-cfg]${NC}"
    exit 1
fi

if [[ ! -f "$REPO_LIST" ]]; then
    echo -e "${RED}❌ Repository list file not found: $REPO_LIST${NC}"
    exit 1
fi

mkdir -p "$INSTALL_DIR"
mkdir -p "$VENV_DIR"

echo -e "${CYAN}📦 Starting repository setup..."
echo -e "📁 Repo install dir: ${INSTALL_DIR}"
echo -e "🐍 Virtualenv dir: ${VENV_DIR}"
echo -e "🛠️  setup.cfg patching: $([[ "$PATCH_CFG" == true ]] && echo "ENABLED" || echo "DISABLED")${NC}"

# Function to patch setup.cfg
update_setup_cfg_executable() {
    local dir="$1"
    local venv_python="$2"
    local cfg_files
    cfg_files=$(find "$dir" -name "setup.cfg")

    for cfg in $cfg_files; do
        echo -e "${CYAN}✏️  Updating 'executable' in $cfg${NC}"
        awk -v python_path="$venv_python" '
        BEGIN { in_section=0 }
        /^\[build_scripts\]/ { in_section=1; print; next }
        /^\[/ { in_section=0; print; next }
        in_section && /^executable *=/ {
            print "executable = " python_path; next
        }
        { print }
        ' "$cfg" > "${cfg}.tmp" && mv "${cfg}.tmp" "$cfg"
    done
}

declare -A cloned_repos
declare -A ran_top_setup

<<<<<<< HEAD
while read -r repo_url package_name venv_name branch; do
=======
while read -r repo_url package_name venv_name; do
>>>>>>> ros2-dev
    if [[ -z "$repo_url" || -z "$package_name" || -z "$venv_name" ]]; then
        echo -e "${YELLOW}⚠️  Skipping invalid line: '$repo_url $package_name $venv_name'${NC}"
        continue
    fi

    repo_name=$(basename "$repo_url" .git)
    clone_path="$INSTALL_DIR/$repo_name"

    # Clone repo only once
<<<<<<< HEAD
    if [[ -z "${cloned_repos[$repo_url]+set}" ]]; then
=======
    if [[ -z "${cloned_repos[$repo_url]}" ]]; then
>>>>>>> ros2-dev
        echo -e "\n${CYAN}🔧 Cloning repo: $repo_name${NC}"
        if [[ -d "$clone_path/.git" ]]; then
            echo -e "${YELLOW}➡️  Already cloned at $clone_path. Skipping clone.${NC}"
        else
<<<<<<< HEAD
            if [[ -n "$branch" ]]; then
                echo -e "${CYAN}📎 Checking out branch: $branch${NC}"
                git clone --branch "$branch" --recurse-submodules "$repo_url" "$clone_path"
            else
                git clone --recurse-submodules "$repo_url" "$clone_path"
            fi
            if [[ -z "$repo_url" || -z "$package_name" || -z "$venv_name" ]]; then
                echo -e "${YELLOW}⚠️  Skipping invalid line: '$repo_url $package_name $venv_name'${NC}"
                continue
            fi
=======
            git clone "$repo_url" "$clone_path"
>>>>>>> ros2-dev
            if [[ $? -ne 0 ]]; then
                echo -e "${RED}❌ Failed to clone $repo_url. Skipping.${NC}"
                continue
            fi
        fi
        cloned_repos[$repo_url]=1
    fi

    # Check for top-level setup.sh
    ran_top_setup["$repo_name"]=false
    top_level_setup="$clone_path/setup.sh"
    if [[ -f "$top_level_setup" ]]; then
        echo -e "${CYAN}🚀 Running top-level setup.sh in $repo_name...${NC}"
        chmod +x "$top_level_setup"
<<<<<<< HEAD
        (cd "$clone_path" && ./setup.sh) || {
            echo -e "${RED}❌ setup.sh failed in $repo_name. Aborting.${NC}"
            exit 1
        }
=======
        (cd "$clone_path" && ./setup.sh)
>>>>>>> ros2-dev
        ran_top_setup["$repo_name"]=true
    fi

    if [[ "$package_name" == "*" ]]; then
        # Find all packages (directories with package.xml)
        package_dirs=$(find "$clone_path" -type f -name "package.xml" -exec dirname {} \;)
    else
        package_dirs=$(find "$clone_path" -type d -name "$package_name" -exec test -f "{}/package.xml" \; -print)
        if [[ -z "$package_dirs" ]]; then
            echo -e "${RED}❌ Package '$package_name' not found under $clone_path${NC}"
            continue
        fi
    fi

    for package_path in $package_dirs; do
        # Run package-level only if top-level wasn't run
        if [[ "${ran_top_setup["$repo_name"]}" != true ]]; then
            if [[ ! -d "$package_path" ]]; then
                echo -e "${RED}❌ Package directory not found: $package_path${NC}"
                continue
            fi

            pkg_name=$(basename "$package_path")
            venv_path="$VENV_DIR/$venv_name"
            venv_python="$venv_path/bin/python3"

            echo -e "\n${CYAN}📦 Setting up package: $pkg_name in env: $venv_name${NC}"

            if [[ -d "$venv_path" ]]; then
                echo -e "${YELLOW}💡 Virtualenv '$venv_name' already exists. Skipping creation.${NC}"
            else
                echo -e "${GREEN}🐍 Creating virtualenv: $venv_path${NC}"
                python3 -m venv "$venv_path"
            fi

            if $PATCH_CFG; then
                update_setup_cfg_executable "$package_path" "$venv_python"
            fi

            setup_script="$package_path/setup.sh"
            if [[ -f "$setup_script" ]]; then
                echo -e "${CYAN}🚀 Running setup.sh for package $pkg_name...${NC}"
                chmod +x "$setup_script"
<<<<<<< HEAD
                (cd "$package_path" && ./setup.sh) || {
                    echo -e "${RED}❌ setup.sh failed in $pkg_name. Aborting.${NC}"
                    exit 1
                }
=======
                (cd "$package_path" && ./setup.sh)
>>>>>>> ros2-dev
            else
                echo -e "${YELLOW}⚠️  No setup.sh found in $package_path${NC}"
            fi
        fi
    done

done < "$REPO_LIST"

echo -e "\n${GREEN}✅ All repositories processed.${NC}"
