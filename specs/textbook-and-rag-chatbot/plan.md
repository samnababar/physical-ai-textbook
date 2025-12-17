# Implementation Plan: AI-Native Technical Textbook

**Branch**: `feat/textbook-and-rag-chatbot`
**Date**: 2025-12-05
**Spec**: `specs/textbook-and-rag-chatbot/spec.md`

## Summary

This project will create a Docusaurus-based online textbook for "Physical AI & Humanoid Robotics". It will include a RAG-based chatbot to answer student questions based on the book's content. The backend will be FastAPI, with Qdrant and Neon Postgres for the RAG pipeline. The entire project will be prepared for deployment on GitHub Pages.

## Technical Context

**Language/Version**: JavaScript (Docusaurus), Python 3.11 (FastAPI)
**Primary Dependencies**: Docusaurus, React, FastAPI, Qdrant, Neon Postgres (psycopg2)
**Storage**: Qdrant (Vector DB), Neon Postgres (Metadata), Markdown files (Content)
**Testing**: Jest (Docusaurus), Pytest (FastAPI)
**Target Platform**: Web (GitHub Pages)
**Project Type**: Web application (frontend + backend)

## Project Structure

### Documentation

```text
specs/textbook-and-rag-chatbot/
├── spec.md
├── tasks.md
└── plan.md
```

### Source Code

```text
.
├── textbook/              # Docusaurus project
│   ├── docs/              # Textbook content (Markdown)
│   │   ├── intro.md
│   │   ├── ch1-ros2/
│   │   ├── ch2-digital-twin/
│   │   ├── ch3-nvidia-isaac/
│   │   └── ch4-vla/
│   ├── src/
│   │   ├── components/
│   │   └── theme/         # Swizzled components, including Chatbot
│   └── docusaurus.config.js
├── backend/               # FastAPI project
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py        # FastAPI app
│   │   ├── api/           # API endpoints (e.g., /chat)
│   │   └── services/      # RAG logic, Qdrant/Postgres clients
│   ├── tests/
│   ├── Dockerfile
│   └── requirements.txt
└── .github/
    └── workflows/
        └── deploy.yml     # GitHub Actions for Docusaurus deployment
```

**Structure Decision**: The project is divided into a `textbook` directory for the Docusaurus frontend and a `backend` directory for the FastAPI application. This separation aligns with the "Web application" structure, promoting a clean separation of concerns between the content presentation and the AI logic.

## Implementation Phases

### Phase 1: Docusaurus Setup (Tasks T001-T004)
1.  Run `npx create-docusaurus@latest textbook classic` to scaffold the site.
2.  Modify `docusaurus.config.js` to set the title to "Physical AI & Humanoid Robotics", a relevant tagline, and configure the deployment options for GitHub Pages (`organizationName`, `projectName`, `deploymentBranch`).
3.  Create the chapter directories inside `textbook/docs/` as specified in the structure.
4.  Add `_category_.json` to each chapter directory to define its label and position in the sidebar.
5.  Create empty markdown files for each week's content (e.g., `week-1.md`).

### Phase 2: Content Generation (Tasks T005-T009)
1.  For each chapter's markdown files, generate placeholder technical content relevant to the chapter's topic (ROS 2, Gazebo, etc.).
2.  Content should be structured with headings, paragraphs, and lists.
3.  Insert placeholder images (e.g., using a service like `https://via.placeholder.com`) for diagrams.
4.  Add formatted code blocks for different languages (e.g., Python, C++, shell).

### Phase 3: RAG Backend (Tasks T010-T015)
1.  Create the `backend` directory and a virtual environment.
2.  Install `fastapi`, `uvicorn`, `qdrant-client`, `psycopg2-binary`, and `pytest`.
3.  Structure the FastAPI app within a subdirectory (`app/`) to follow best practices.
4.  In `app/main.py`, define a `/chat` endpoint using `@app.post("/chat")`. It will take a JSON body with a "query" field.
5.  Initially, the `/chat` endpoint will return a hardcoded JSON response for testing purposes (e.g., `{"answer": "This is a mock response."}`).
6.  Create empty files `app/services/qdrant_client.py` and `app/services/postgres_client.py` to house future logic.
7.  Write a basic `Dockerfile` that installs dependencies and runs the FastAPI app with `uvicorn`.

### Phase 4: RAG Frontend (Tasks T016-T019)
1.  Use the Docusaurus swizzle command to eject the `Layout` component: `npm run swizzle @docusaurus/theme-classic Layout -- --eject`.
2.  Create a new file `src/theme/Chatbot/index.js` and `src/theme/Chatbot/styles.css` for the chatbot component.
3.  The `Chatbot` component will be a simple React component with state for the input text, conversation history, and loading status. It will render a button to open a modal/chat window.
4.  Modify the swizzled `src/theme/Layout/index.js` to import and render the `Chatbot` component, making it globally available.
5.  Implement an `async` function inside the `Chatbot` component that uses the `fetch` API to send a POST request to the backend's `/chat` endpoint (e.g., `http://localhost:8000/chat`).

### Phase 5 & 6: Bonus Features & Deployment (Tasks T020-T025)
1.  The i18n configuration in `docusaurus.config.js` will be set up with `en` and `ur` locales. The directory `i18n/ur/` will be created to hold translated markdown.
2.  A GitHub Actions workflow file will be created at `.github/workflows/deploy.yml`. It will use the standard `peaceiris/actions-gh-pages` action to build the Docusaurus site from the `textbook` directory and deploy the `build` output to the `gh-pages` branch.
3.  A comprehensive `README.md` will be written at the project root.
