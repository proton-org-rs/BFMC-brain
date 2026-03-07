#!/bin/bash

# Script to change the Brain git repository to the new URL
# Usage: ./change-repo.sh

NEW_REPO_URL="https://github.com/proton-org-rs/BFMC-brain"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "Changing Brain Git Repository"
echo "========================================"
echo "New Repository: $NEW_REPO_URL"
echo "Working Directory: $SCRIPT_DIR"
echo ""

# Change to Brain directory
cd "$SCRIPT_DIR" || exit 1

# Show current remote
echo "Current remote URL:"
git remote -v
echo ""

# Check for uncommitted changes
echo "Checking for uncommitted changes..."
if ! git diff-index --quiet HEAD --; then
    echo "⚠ You have uncommitted changes. Please commit them first:"
    git status
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
else
    echo "✓ No uncommitted changes"
fi

echo ""

# Create a backup branch with current state
echo "Creating backup branch with current state..."
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
BACKUP_BRANCH="backup-before-repo-change-$(date +%s)"
git branch "$BACKUP_BRANCH"
echo "✓ Backup branch created: $BACKUP_BRANCH"
echo ""

# Change the remote URL
echo "Updating remote URL..."
git remote set-url origin "$NEW_REPO_URL"

# Verify the change
echo ""
echo "New remote URL:"
git remote -v
echo ""

# Fetch from new repository
echo "Fetching from new repository..."
git fetch origin

# Show available branches
echo ""
echo "Available branches in new repository:"
git branch -r
echo ""

echo "Your local code is safe. To push your changes to the new repository:"
echo "  git push origin $CURRENT_BRANCH"
echo ""
echo "If you want to sync with the new repo's main branch:"
echo "  git pull origin main"

echo ""
echo "========================================"
echo "✓ Repository change complete!"
echo "========================================"
