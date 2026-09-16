"""
push_to_github.py
=================
Automated Git Push Utility for SteadyPath Repository

Usage:
    .\.venv\Scripts\python.exe push_to_github.py [GITHUB_TOKEN]

If GITHUB_TOKEN is not provided as a command-line argument or env var (GITHUB_TOKEN),
it will prompt securely for the token.
"""

import os
import sys
import getpass
from dulwich import porcelain
from dulwich.repo import Repo

REPO_URL = "https://github.com/basappababanagar911-web/snehal.git"

def main():
    token = os.environ.get("GITHUB_TOKEN")
    if len(sys.argv) > 1:
        token = sys.argv[1].strip()

    if not token:
        print("=" * 60)
        print("GitHub Authentication Required for Push")
        print("Target: " + REPO_URL)
        print("=" * 60)
        token = getpass.getpass("Enter your GitHub Personal Access Token (PAT): ").strip()

    if not token:
        print("[ERROR] No token provided. Aborting push.")
        sys.exit(1)

    auth_url = f"https://oauth2:{token}@github.com/basappababanagar911-web/snehal.git"
    
    repo = Repo(".")
    print(f"Pushing branch 'main' to {REPO_URL}...")
    try:
        porcelain.push(repo, auth_url, "refs/heads/main:refs/heads/main")
        print("\n[SUCCESS] All SteadyPath commits and artifacts successfully pushed to GitHub!")
        print(f"Repository URL: {REPO_URL}")
    except Exception as e:
        print(f"\n[ERROR] Push failed: {e}")
        print("Please ensure your Personal Access Token has 'repo' (read/write) permissions.")
        sys.exit(1)

if __name__ == "__main__":
    main()
