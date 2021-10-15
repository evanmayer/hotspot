# hotspot
Code for simulating and driving a cable-driven parallel robot for mapping beams on surfaces.

## Running

When a command line interface is established, it will be described here.

### Dependencies

#### Creating a conda environment
I use [miniconda](https://docs.conda.io/en/latest/miniconda.html) to keep my development environments separate. If there is a .yml file available that lists the dependencies I need, I run `conda env create -f <filename>.yml` to use it to make a new conda env with those dependencies. Conda parses the contents, creates a new env with the given name, and installs the required packages.

#### Updating .yml as new dependencies are needed
I periodically use `conda env export --from-history | tee hotspot.yml` to dump whatever I've manually installed so far into the .yml. This avoids specifying exact versions of packages, but will still try to install all the packages needed.

## Contributing

### Testing
Testing is accomplished with `pytest`. Passing tests are a prerequisite for committing code, and new code should come with new tests.

To run the test suite defined in the `tests` dir, change dir to the toplevel dir of the repo and execute 
```
pytest
```

### Pushing changes
If you need push permissions, message me. Otherwise, you may fork, create a new branch with your changes, and fill out a Pull Request to have the changes reviewed and merged in.

### How did you make your flowcharts?
[diagrams.net](https://diagrams.net)
