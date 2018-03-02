# FCND-Term1-P1-Backyard-Flyer
Udacity Flying Car Nanodegree - Term 1 - Project 1 - Backyard Flyer


# Prerequisites

In order to run this project you need to have the following software installed:

- [Miniconda](https://conda.io/miniconda.html) with Python 3.6. I had some problems while installing this on my Mac after having an older version install and some other packages install with Homebrew. I have to manually delete all the `~/*conda*` directory from my home and then install it with `bash Miniconda3-latest-MacOSX-x86_64.sh -b`.
- [Udacity FCND Simulator](https://github.com/udacity/FCND-Simulator-Releases/releases) the latest the better.

# Run the code
Just in case you don't have it, update to the latest conda version:
```
conda update -n base conda
```
At this time, the latest version is 4.4.11

Change directory to where you clone this repo. Let's call that directory REPO_PATH. Create the conda environment for this project:
```
conda env create -f environment.yml
```
**Note**: This environment configuration is provided by Udacity at [the FCND Term 1 Starter Kit repo](https://github.com/udacity/FCND-Term1-Starter-Kit).

Activate your environment with the following command:
```
source activate fcnd
```
