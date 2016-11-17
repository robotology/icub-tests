icub-tests
==========
The `icub-tests` repository contains tests for the iCub robot.

See documentation at https://robotology.github.io/icub-tests/

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

## Contributors

- [**Ali Paikan**](https://github.com/apaikan)
- [**Lorenzo Natale**](https://github.com/lornat75)
- [**Silvio Traversaro**](https://github.com/traversaro)
- [**Alessandro Scalzo**](https://github.com/ale-git)
- [**Marco Randazzo**](https://github.com/randaz81)
- [**Valentina Gaggero**](https://github.com/valegagge)
- [**Ugo Pattacini**](https://github.com/pattacini)

