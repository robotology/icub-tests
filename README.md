icub-tests
==========
The `icub-tests` repository contains tests for the iCub robot.

See documentation on https://robotology.github.io/icub-tests/

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

* Ali Paikan 
* Lorenzo Natale
* Silvio Traversaro
* Alessandro Scalzo
* Marco Randazzo

