# RAG Chatbot Documentation

This documentation site is built with [Docusaurus 2](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
cd docs
npm install
```

## Local Development

```bash
cd docs
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
cd docs
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

The documentation is automatically deployed to GitHub Pages when changes are pushed to the `main` branch using the GitHub Actions workflow defined in `.github/workflows/deploy.yml`.