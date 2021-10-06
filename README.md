# hotspot
Code for simulating and driving a cable-driven parallel robot for mapping beams on surfaces.

## Dependencies

### Creating a conda environment
I use (miniconda)[https://docs.conda.io/en/latest/miniconda.html] to keep my development environments separate. If there is a .yml file that lists the dependencies I need, I run `conda env create -f <filename>.yml` to use it to make a new conda env with those dependencies.

### Updating .yml as new dependencies are needed
I periodically use `conda env export --from-history | tee hotspot.yml` to dump whatever I've manually installed so far into the .yml. This avoids specifying exact versions of packages, but will still try to install all the packages needed.
