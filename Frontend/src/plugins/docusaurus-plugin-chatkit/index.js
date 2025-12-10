module.exports = function(context, options) {
  return {
    name: 'docusaurus-plugin-chatkit',
    injectHtmlTags() {
      return {
        postBodyTags: [
          `<div id="chatkit-container"></div>`,
        ],
      };
    },
  };
};