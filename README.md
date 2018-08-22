This repository is part of the Neurorobotics Platform software  
Copyright (C) Human Brain Project  
https://neurorobotics.ai

The Human Brain Project is a European Commission funded project
in the frame of the Horizon2020 FET Flagship plan.  
http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships

You are free to clone this repository and amend its code with respect to
the license that you find in the root folder.

## Submitting a pull request

To submit code changes (pull requests), do as follows.  
0. Log in to Bitbucket  
1. Clone the repo : click the "Clone" button in the top right of the Source tree view and
   copy-paste the command in a terminal (use HTTPS for simplicity)  
2. Create a branch in Bitbucket : "+" button, create a branch. Give it an explicit name
   without spaces or special characters. Your change should refer to a ticket in Jira, that
   you or someone else created for this change. Then embed the ticket number in the branch
   name. Example: NUIT-10_my_new_feature  
3. Copy paste the git fetch command from Bitbucket to your terminal to get the branch
   locally  
4. Do your code changes  
5. git add whatever files you changed/added  
6. git commit and **make sure** your commit message starts with [<ticket_number>].
   Example: "[NUIT-10] My new feature"  
7. git push (you will be asked for your Bitbucket credentials)  
6. Check the build of your commit in the Pipelines menu in Bitbucket (filter your branch)  
7. You can do as many commits and push to your branch as needed  
8. When ready to get reviewed, create a pull request : click on "+" in Bitbucket and
   "Create a pull request". Select your branch and add a description  
9. A core developer will eventually review your pull request and approve or comment it  
