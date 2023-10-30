# Git Tutorial

Git is a version control system (VCS) for tracking changes in computer files and coordinating work on those files among multiple people.

## Instalation of git

First of all, obviously, before using git, you must have git installed. [Here](https://github.com/git-guides/install-git) are the steps to do it.



## Step by step using git

1. **Create a new Directory**

```shell
mkdir git_tutorial
cd git_tutorial 
```

2. **Start a git project**

```shell
git init # Initializes the git repository
```

Now the git repository is created. It is time to test.

3. **The tracking of the files**

To track the files of the repository, the command to use is 

```shell
git status
```

As you do not have any tracked files, the output is not that interesting. Let's create our first file.

```shell
touch fs.txt
```

Now, if you run `git status` again, the output will be more interesting, showing the file `fs.txt` was created.

4. **The staging area**

The staging area is the tracked files that were added there. It has the files that will be commited later. To add a file to the staging area, a command must be used. Let's use it with `fs.txt`.

```shell
git add fs.txt
```

Alternivaly, if you want to add all of the files to the staging area, we can use the command

```shell
git add -A
```

5. **Our first commit**

Now that we have the tracked files into the staging are, let's make our first commit. In Git, a commit refers to a snapshot of the changes made to files in a repository at a specific point in time. Each commit is accompanied by a unique identifier and a commit message that describes the changes made. To commit our changes we use:

```shell
git commit -m <commit message>
```

In our case, we will commit like this:

```shell
git commit -m "Added the fs.txt file"
```


<p align="center">
  <img src="../assets/git_tutorial/git1.png" style="width: 25%; height: auto;">
</p>

Your commits should look like this. We have our commit represented as a bubble. The `master` is the default branch that we are using to develop (more details about branches will be explainded later). The `HEAD` is only pointing to the branch that we are developing.

6. **Tracking the commits**

If you want to check the last commits that were made, you can use:

```shell
git log
```

7. **Another commit**

Now, let's modify the `fs.txt` and do all the same process.

```shell
echo "Autonomous Systems" > fs.txt
git add -A
git commit -m "Wrote fs.txt"
git log
```

<p align="center">
  <img src="../assets/git_tutorial/git2.png" style="width: 50%; height: auto;">
</p>

Now you have two commits that track the same file, `fs.txt`.

8. **Creation of Branches**

A Git branch is a parallel version of a repository that allows you to work on different aspects of a project simultaneously, enabling you to make changes without affecting the main codebase until you're ready to merge your work.

We can create a branch this way:

```shell
git branch <branch-name>
```

In our case, we will create a branch named `documentation`:

```shell
git branch documentation
```

Then, to start developing on that branch, we use the following command to change the branch:

```shell
git checkout documentation
```

Eventually, if you want to list all the branches, just use the following command:

```shell
git branch
```

9. **Commit to the branch**

Now, follow the procedures of the seventh step:

```shell
echo "Perception" > fs2.txt
git add fs2.txt
git commit -m "Wrote fs2.txt"
```
<p align="center">
  <img src="../assets/git_tutorial/git3.png" style="width: 70%; height: auto;">
</p>

In the figure, we can see now that we not only have the `master` branch that points to a commit, but also we have a `documentation` branch that points to our very last commit.

Then, go back to the master branch and make a small commit like this:

```shell
git checkout master
echo "Planning" > fs3.txt
git add fs3.txt
git commit -m "Wrote fs3.txt"
```

<p align="center">
  <img src="../assets/git_tutorial/git4.png" style="width: 70%; height: auto;">
</p>

Now, if you check the commits with `git log`, you can see that the commit `Wrote fs2.txt` you created isn't there. That occurs, because the commit is in the other branch.

You may test commiting on your branch to garantee that you're conforable with that.

10. **Merging Branches**

Now that you have made all of the changes to your file and you're secure it is functional, you can do an action that is called `merge`. This action will merge the two branches.

```shell
git checkout master
git merge documentation
```

<p align="center">
  <img src="../assets/git_tutorial/git5.png" style="width: 100%; height: auto;">
</p>

With this, the master branch will be updated with the content that was added in the documentation branch. 

Note that this command differes from:

```shell
git checkout documentation
git merge master
```

In this case, the documentation branch is the one that is updated. The master branch keeps the same.

11. **Integration with a remote repository**

Now, we will integrate our local repository with a remote repository, we will the GitHub. With GitHub, several people can interact with the same

[Here](https://youtu.be/iWs34DO_H2M?feature=shared) is a step by step of how to integrate your machine with Github.

12. **Push the repository to Github**

Next, we must configure our local repository with the remote one. First, we will be using

```shell
git remote add origin git@github.com:<your username>/<repository name>.git
```

Then, it's just use the command `push` to update the Github repository with our local changes.

```shell
git push
```

## Our Repository Context

If you already are confortable with git, the next step is to follow [this tutorial](../compile-test-run.md). Where there are all the tips to clone the project, install its dependencies, compile the code and to run and test the code. However here are some advices:

1. **Commit Rules**

There are git norms that must be respected. They can be found [here](../project-rules.md#commits).

2. **Pull Requests**

Commit directly to the main (dev) branch must be avoided. Instead you must create a branch for your development and create a pull request for review when you think that your work is done and can be added to the dev branch.

The pull requests can be done on GitHub and must be attached with the ClickUp the following way:

Every Pull resquest must have a reviewer that have to approve the pull request. Also, the pull request must have the approvation of the department leader. When both reviewers approve the pull request, it can be merged.

## More information

This tutorial was based on the slides of Prof. André Restivo. You can find them [here](https://paginas.fe.up.pt/~arestivo/slides/?s=git#1) if you want more precise information about how git works.

