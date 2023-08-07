# Contributing

This repo is maintained by TDK Invensense, and write access to branches is
restricted. We still welcome your contributions to the project. Please read
below to see how you can contribute.

## Authorship

It is important to include authorship information in the source files if
you want it to be preserved. This repo is maintained by squash-merging commits
from an internal repo, so inidivual commit messages will be lost or modified.

You can simply include lines like the following on a per-file basis:

```
// Contributors: <Author 1> <Author 2>
// Copyright: (c) <year> <Author 1>, (c) <year> <Author 2>
```

Or you can include it at a finer scope if you wish.

## Formatting

This project is set up to use clang-format to ensure consistent formatting of
the source code. You can install this using python.

```
python -m pip install clang-format
```

You can run clang-format locally using the following command to check for issues:

```
clang-format --files=format_flist.txt --dry-run -Werror
```

To fix issues, run:

```
clang-format --files=format_flist.txt -i
```

If there are special cases where you would like to disable clang-format, you can
surround your code with:

```c
// clang-format off
...
// clang-format on
```

This will prevent clang-format from touching any code between the off and on
commands. Currently this is used to preserve markdown tables within doxygen
block comments.

## Contribution flow

To start contributing, you will first need to fork this repository. From you
fork, you can create branches, make commits, etc. without restriction.

When you would like to contribute something back to this repo, you should open a
pull request to the `master` branch. From here, a TDK Invensense member will
review the code and decide whether to accept the contribution or not. In either
case, the PR will not be directly merged.

If the contribution is accepted, it will first be integrated into our internal
repo and put through our normal release flow. When the changes are available
in the TDK Invensense repo, you can then merge or rebase these changes into your
fork.

Once the changes are released or rejected by TDK Invensense, the original PR
will be closed.
