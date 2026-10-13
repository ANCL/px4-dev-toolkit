/* MathJax 3.2.2 with Arithmatex; full-page navigation needs no SPA hooks.
 * Configuration: https://squidfunk.github.io/mkdocs-material/reference/math/
 */
window.MathJax = {
  tex: {
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex"
  }
};
