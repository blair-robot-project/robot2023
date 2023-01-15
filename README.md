# robot2023
Our code for our 2023 robot *TBA* for the FRC Charged Up Game!

---

## Workflows

There are currently three workflows, `run-tests.yml` to run tests, `ktlint.yml` to check formatting, and `gen-docs.yml` to generate documentation. Here is how `gen-docs.yml` works:

- It's triggered whenever you push to `main`
- It runs `./gradlew dokkaHtml` to generate HTML from all our doc comments
- The generated HTML is uploaded and deployed to GitHub Pages
- The documentation is then accessible at https://blair-robot-project.github.io/framework2022/.
