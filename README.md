icub-tests
==========
The `icub-tests` repository contains tests for the iCub robot.
Tests are written using the [Robot Testing Framework](https://github.com/robotology/robot-testing) (RTF).

## Installation, tutorial and available tests

See the [icub-test documentation](https://robotology.github.io/icub-tests/).

## Update the documentation

To update documentation follow these steps:

```bash
 git checkout gh-pages
 git rebase master
 cd doxygen
 rm -rf doc
 doxygen ./generate.txt
 git add ./doc
 git log -1
 git commit --amend
 git push --force-with-lease
 git checkout master
```
For in-depth details about doxygen documentation and gh-pages, see [how-to-document-modules](https://github.com/robotology/how-to-document-modules).

## Contributors

- [**Ali Paikan**](https://github.com/apaikan)
- [**Lorenzo Natale**](https://github.com/lornat75)
- [**Silvio Traversaro**](https://github.com/traversaro)
- [**Alessandro Scalzo**](https://github.com/ale-git)
- [**Marco Randazzo**](https://github.com/randaz81)
- [**Valentina Gaggero**](https://github.com/valegagge)
- [**Ugo Pattacini**](https://github.com/pattacini)
- [**Nicolo' Genesio**](https://github.com/Nicogene)
