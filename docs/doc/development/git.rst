How to git
==========
.. inclusion-introduction-start

This tutorial will describe the current git based workflow we adhere to.

.. inclusion-introduction-end

Prerequisites
^^^^^^^^^^^^^
Before following the following steps make sure you:

*  Have git installed on your device.
*  Have git LFS (Large File Storage) installed on your device.

    * To install see: https://git-lfs.github.com/
    * To check if you have it installed run: :code:`dpkg -l | grep -E '^ii' | grep lfs`
*  Have your git config setup.

    * :code:`git config --global user.name "[Your name]"`
    * :code:`git config --global user.email "[Your email]"`
* Have cloned the march repo.

Roadmap for adding something to the repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Create an issue on gitlab.
    #. Go to issues on the gitlab page (https://gitlab.com/project-march/march/-/issues)
    #. Press the blue button with "New issue".
    #. Give a clear title.
    #. Choose a template for description.
    #. Fill in the rest of the description.
    #. Assign it to yourself
    #. Add the relevant titles
    #. Press the blue button at the bottom with "Create issue".
#. Create a branch via the issue
    #. Press the arrow in the blue button "Create merge request".
    #. Select "Create branch".
    #. (Optional) Select a different Source branch. Keep it on "dev" if you are unsure.
#. Fetch the new repo data in your cloned folder.
    #. Open a terminal with git installed.
    #. cd to the cloned "march" folder.
    #. Type :code:`git fetch`.
#. Checkout/switch to your new branch
    * :code:`git checkout [issue_number]-[issue_name]` (e.g. :code:`git checkout 1103-fix-docs-bug`)
    * :code:`git switch [issue_number]-[issue_name]` (e.g. :code:`git switch 1103-fix-docs-bug`)
#. Start adding code. :) #code_knallen
#. Commit your code.
    **Note: Make sure your code is still runnable after every commit. If this is not the case for some reason
    make sure you add "draft:" at the start of your commit message.**

    #. Type :code:`git status` to see which files have changed.
    #. Type :code:`git add [changed files you wish to commit]`
    #. Type :code:`git status` again to check if all the correct files are staged.
    #. Type :code:`git commit -m "[Commit message]"`

    **Note: The commit message should be active form and should follow from: "This commit will [Commit message]**.
#. Push all your commit
    #. Type :code:`git push`.
#. Create a merge request via gitlab
    #. Go to the merge requests page. (https://gitlab.com/project-march/march/-/merge_requests)
    #. Select the blue button "New merge request".
    #. Select as source branch your branch that you where working from
    #. By default keep dev as target branch.
    #. Press the blue button "Compare branches and continue".
    #. Write a detailed overview of what this commit does.
    #. Assign at least 2 reviewers,
        * One (or more) of the software architects.
            * George Vegelien
            * Jelmer de Wolde
            * Tuhin Das
        * Someone from semi-software that has knowledge of the issue.
    #. (Default) Leave "Delete source branch when merge request is accepted." checked.
        Only uncheck this one if you wish to keep working on the feature.
    #. (Default) Leave "Squash commits when merge request is accepted." unchecked.
        Only check this one if the branch has an ugly commit history.
#. Check if the pipeline succeeds, otherwise fix those issues.
    This can be found on:

    * The CI/CD tab on gitlab.
    * The merge request page.
#. Check for merge conflicts. If there are any, resolve those.
#. Wait for the MR (merge request) to be reviewed and merged. :)


