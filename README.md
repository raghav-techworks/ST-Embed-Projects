# rd-embed-Firmware
Contains the projects related microcontrollers


Setting up git hub account on windows
Step 1: Install Git on Windows
    1. Download Git for Windows from git-scm.com.
    2. Run the installer and follow the setup process:
        Choose "Git from the command line and also from 3rd-party software".
        Choose "Use the OpenSSH bundled with Git".
        Choose "Use the default native Windows Secure Channel library".
        Choose "Checkout Windows-style, commit Unix-style line endings" (recommended).
        Choose "Use MinTTY (default terminal for Git Bash)".
        Complete the installation and restart your terminal.

Step 2: Verify Git Installation
    Open Git Bash and run:
         -------------------      
         | $git --version  |
         -------------------
    If Git is installed, it will show the version number.

Step 3: Configure Git
    Set up your username and email:
         -------------------------------------------
        | git config --global user.name "Your Name" |
         -------------------------------------------
         ---------------------------------------------------------
        | git config --global user.email "your-email@example.com" |
         ---------------------------------------------------------
Verify the configuration:
         -------------------
        | git config --list |
         -------------------

Step 4: Generate SSH Key and Add to GitHub
    1. Generate an SSH key:
         -------------------------------------------------------
        | ssh-keygen -t rsa -b 4096 -C "your-email@example.com" |
         -------------------------------------------------------
    Press Enter to save it in the default location.

    2. Start the SSH agent:
         ------------------------
        | eval "$(ssh-agent -s)" |
         ------------------------
    
    3. Add your SSH key to the agent:
         -----------------------
        | ssh-add ~/.ssh/id_rsa |
         -----------------------
    
    4. Copy the SSH key to the clipboard:
         --------------------------
        | clip < ~/.ssh/id_rsa.pub |
         --------------------------
    (If clip is not found, open the file manually: | cat ~/.ssh/id_rsa.pub | and copy it.)

    5. Add the SSH key to your GitHub account:
         --------------------------------------------
        | Go to GitHub → Settings → SSH and GPG keys. |
         --------------------------------------------
    Click New SSH Key, paste the key, and save.

    6. Test SSH connection:
         -----------------------
        | ssh -T git@github.com |
         -----------------------
    You should see a success message: "Hi username! You've successfully authenticated, but GitHub does not provide shell access."

Step 5: Clone Your Repository
    Navigate to the folder where you want to clone your repository:
         ------------------------
        | cd path/to/your/folder |
         ------------------------
    Clone the repository (replace your-repo-name with the actual repository name):
         -----------------------------------------------------------
        | git clone git@github.com:your-username/your-repo-name.git |
         -----------------------------------------------------------

