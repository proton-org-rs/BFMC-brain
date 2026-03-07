# Script to change the Brain git repository to the new URL
# Usage: .\change-repo.ps1

$NEW_REPO_URL = "https://github.com/proton-org-rs/BFMC-brain"
$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "========================================" -ForegroundColor Green
Write-Host "Changing Brain Git Repository" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host "New Repository: $NEW_REPO_URL"
Write-Host "Working Directory: $SCRIPT_DIR"
Write-Host ""

# Change to Brain directory
Set-Location $SCRIPT_DIR

# Show current remote
Write-Host "Current remote URL:" -ForegroundColor Cyan
git remote -v
Write-Host ""

# Check for uncommitted changes
Write-Host "Checking for uncommitted changes..." -ForegroundColor Yellow
$status = git status --porcelain
if ($status) {
    Write-Host "⚠ You have uncommitted changes:" -ForegroundColor Yellow
    git status
    Write-Host ""
    $response = Read-Host "Continue anyway? (y/n)"
    if ($response -ne "y" -and $response -ne "Y") {
        Write-Host "Aborted." -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "✓ No uncommitted changes" -ForegroundColor Green
}

Write-Host ""

# Create a backup branch with current state
Write-Host "Creating backup branch with current state..." -ForegroundColor Yellow
$CURRENT_BRANCH = (git rev-parse --abbrev-ref HEAD)
$TIMESTAMP = Get-Date -Format "yyyy-MM-dd-HH-mm-ss"
$BACKUP_BRANCH = "backup-before-repo-change-$TIMESTAMP"
git branch $BACKUP_BRANCH
Write-Host "✓ Backup branch created: $BACKUP_BRANCH" -ForegroundColor Green
Write-Host ""

# Change the remote URL
Write-Host "Updating remote URL..." -ForegroundColor Yellow
git remote set-url origin $NEW_REPO_URL

# Verify the change
Write-Host ""
Write-Host "New remote URL:" -ForegroundColor Cyan
git remote -v
Write-Host ""

# Fetch from new repository
Write-Host "Fetching from new repository..." -ForegroundColor Yellow
git fetch origin

# Show available branches
Write-Host ""
Write-Host "Available branches in new repository:" -ForegroundColor Cyan
git branch -r
Write-Host ""

Write-Host "Your local code is safe. To push your changes to the new repository:" -ForegroundColor Green
Write-Host "  git push origin $CURRENT_BRANCH"
Write-Host ""
Write-Host "If you want to sync with the new repo's main branch:" -ForegroundColor Cyan
Write-Host "  git pull origin main"

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "✓ Repository change complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
