# framework2022
Framework rewrite after 2022 season

---

## Workflows

There are currently three workflows, `run-tests.yml` to run tests, `ktlint.yml` to check formatting, and `gen-docs.yml` to generate documentation. Here is how `gen-docs.yml` works:

- It's triggered whenever you push to `main`
- It overwrites its local `docs` branch with the `main` branch
- It runs `./gradlew dokkaHtml` to generate HTML from all our doc comments and copies that over to the `docs` folder
- It commits that to the `docs` branch and pushes it
- The documentation is then accessible at https://blair-robot-project.github.io/framework2022/.
