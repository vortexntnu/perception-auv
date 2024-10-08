# Perception AUV
Parent repository including launch files for perception AUV related submodules. 
Implements git submodules to allow for strict version control of package dependencies. 

## Launch
Main launch command to launch all perception AUV related packages.
```bash
# launch with --show-args to print out all available launch arguments
ros2 launch perception_setup perception.launch.py
```
See [perception_setup/README.md](perception_setup/README.md) for more info.

## Dependencies
- [vortex-blackfly-driver](https://github.com/vortexntnu/vortex-blackfly-driver)
- [vortex-image-filtering](https://github.com/vortexntnu/vortex-image-filtering)
- [vortex-aruco-detection](https://github.com/vortexntnu/vortex-aruco-detection)
- [vortex-vkf](https://github.com/vortexntnu/vortex-vkf)
  
## How to work with submodules
A git submodule is nothing more than a repository inside another repository. The submodules are stored as links to specific commits of their respective repositories.

### Cloning
To clone a repository containing submodules you should clone with the `--recurse-submodules` option.

```bash
git clone --recurse-submodules https://github.com/vortexntnu/perception-auv
```
If you pass --recurse-submodules to the git clone command, it will automatically initialize and update each submodule in the repository, including nested submodules if any of the submodules in the repository have submodules themselves.

If you forgot the `--recurse-submodules` option you still get the directories that contain the submodules, but none of the files within them. To fix this just run the following command from the parent directory.
```bash
git submodule update --init --recursive
``` 
  - `init` initializes your local configuration file `.gitmodules` and updates your local `.git/congig` so that Git knows where the fetch the submodules from.
  - `update` clones the submodules repositories and checks out to the commit that this parent repository points to.
  - `recursive` adds support for nested submodules.

### Pulling
By default , the `git pull` command recursively fetches submodule changes. However, it does not **update** the submodules. To finalize the update, you need to run `git submodule update --init --recursive`

### Adding Submodules
Add a new submodule (repository):
```bash
cd packages
git submodule add <repository-url>
```
This command will automatically clone the submodule and create an entry in the `.gitmodules` file.

### Working on Submodules
Once submodules are properly initialized and updated within a parent repository they can be utilized exactly like stand-alone repositories. This means that submodules have their own branches and history. When making changes to a submodule it is important to publish submodule changes and then update the parent repositories reference to the submodule.

After making changes to a submodule, your local submodule(directory) now points to a different commit. The parent repository will still point to a different commit for this submodule.
```bash
$ git submodule status
+bf427ba5b2e59bcce6e4477a71db21eb4b00ca79 packages/vortex-aruco-detection (heads/main)
 fa498271bac3d66bdec8a5096e7ec1cd490d57d9 packages/vortex-blackfly-driver (heads/main)
 5e6a75a749d3f64db3414e1561e81341a4c88edb packages/vortex-image-filtering (heads/main)
 3d371c99dbc1c56dfc3b7879322a44299318026b packages/vortex-vkf (v2.1.0)
```
the `+`-sign in front of the first commit hash listed indicates that 
`+` (plus sign) Listed in front of the first commit hash indicates that the submodule has changes (i.e., it is in a detached HEAD state pointing to a commit that is not the current commit of the branch it is supposed to track). This could happen if the submodule has local modifications or is checked out to a different commit than what is recorded in the parent repository. 

To fix this we need to update the parent repository.
```bash
$ git status
On branch main
Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   packages/vortex-aruco-detection (new commits)
```
Executing git status shows us that the parent repository is aware of the new commits to the awesomelibrary submodule. It doesn't go into detail about the specific updates because that is the submodule repositories responsibility. The parent repository is only concerned with pinning the submodule to a commit. Now we can update the parent repository again by doing a git add and git commit on the submodule. This will put everything into a good state with the local content. If you are working in a team environment it is critical that you then git push the submodule updates, and the parent repository updates.


### Publishing Submodule Changes
**NB!** If we commit in the main project and push it up without pushing the submodule changes up as well, other people who try to check out our changes are going to be in trouble since they will have no way to get the submodule changes that are depended on. Those changes will only exist on our local copy.

To prevent this when pushing the parent directory one should use the following command.
```bash
$ git push --recurse-submodules=check
The following submodule paths contain changes that can
not be found on any remote:
  <submodule_name>

```
This allows us to manually go into the submodules and push to the remotes to make sure they're externally available and then try this push again.

### Switching Branches
When you switch branches, the main repository’s .gitmodules file and submodule commit references might change. This is why the submodules need to be updated.

Newer Git versions (Git >= 2.13) simplify all this by adding the --recurse-submodules flag to the git checkout command, which takes care of placing the submodules in the right state for the branch we are switching to.

```bash
$ git --version
git version 2.13.3

$ git checkout --recurse-submodules master
Switched to branch 'master'
Your branch is up-to-date with 'origin/master'.

$ git status
On branch master
Your branch is up-to-date with 'origin/master'.

nothing to commit, working tree clean
```

If you don't have the required Git version and checkout to a branch with fewer submodules, you still have the submodule directory as an untracked directory.

```bash
$ git --version
git version 2.12.2

$ git checkout master
warning: unable to rmdir <submodule_name>: Directory not empty
Switched to branch 'master'
Your branch is up-to-date with 'origin/master'.

$ git status
On branch master
Your branch is up-to-date with 'origin/master'.

Untracked files:
  (use "git add <file>..." to include in what will be committed)

	<submodule_name>/

nothing added to commit but untracked files present (use "git add" to track)
```

To remove the untrack directory you could manually remove it or run the followin command:

```bash
$ git clean -ffdx
``` 
- f (force): As above, this option is required to force the cleaning of untracked files.
- f (force): The second -f enhances the forcefulness of the clean operation. It is required for -x to take effect.
- d (directories): This option tells Git to remove untracked directories in addition to untracked files.
- x (ignored files): This option tells Git to remove not only untracked files and directories but also files that are ignored by .gitignore.


Be careful with this command as this will also remove all untracked changes.

Finally, to put the submodules in the right state run:

```bash
git submodule update --init --recursive
```