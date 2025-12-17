# Tasks: AI-Native Technical Textbook

**Input**: `specs/textbook-and-rag-chatbot/spec.md`

## Phase 1: Docusaurus Project Setup

**Purpose**: Initialize the Docusaurus project and basic structure for the textbook.

- [ ] T001 Initialize a new Docusaurus project named 'textbook'.
- [ ] T002 Configure `docusaurus.config.js` with the project title, tagline, and GitHub Pages deployment settings.
- [ ] T003 Create the basic folder structure for the four main chapters.
- [ ] T004 [P] Create placeholder markdown files for each weekly breakdown within the chapters.

---

## Phase 2: Textbook Content Generation (User Story 1 - P1)

**Goal**: Populate the textbook with initial content.

**Independent Test**: The Docusaurus site can be built and the content is visible.

- [ ] T005 [P] [US1] Generate content for "Chapter 1: Robotic Nervous System (ROS 2)".
- [ ] T006 [P] [US1] Generate content for "Chapter 2: Digital Twin (Gazebo & Unity)".
- [ ] T007 [P] [US1] Generate content for "Chapter 3: AI-Robot Brain (NVIDIA Isaac)".
- [ ] T008 [P] [US1] Generate content for "Chapter 4: Vision-Language-Action (VLA)".
- [ ] T009 [US1] Add example diagrams and code snippets to the content.

---

## Phase 3: RAG Chatbot Backend (User Story 2 - P2)

**Goal**: Set up the backend service for the RAG chatbot.

**Independent Test**: The FastAPI server can be started and the API endpoints are reachable.

- [ ] T010 [US2] Create a `backend` directory and initialize a Python project with FastAPI.
- [ ] T011 [US2] Add dependencies: `fastapi`, `uvicorn`, `qdrant-client`, `psycopg2-binary`.
- [ ] T012 [US2] Create a main `main.py` file for the FastAPI application.
- [ ] T013 [US2] Implement a placeholder `/chat` endpoint that accepts a query and returns a mock response.
- [ ] T014 [US2] Set up placeholder modules for Qdrant and Neon Postgres integration.
- [ ] T015 [US2] Create a `Dockerfile` for the backend service.

---

## Phase 4: RAG Chatbot Frontend (User Story 2 - P2)

**Goal**: Embed the chatbot UI in the Docusaurus textbook.

**Independent Test**: The chatbot UI is visible and can be interacted with on the textbook pages.

- [ ] T016 [US2] Create a new React component for the chatbot UI in the Docusaurus `src/theme` directory.
- [ ] T017 [US2] The chatbot component should have a text input for questions and a display area for answers.
- [ ] T018 [US2] Swizzle the Docusaurus `Layout` component to include the chatbot UI.
- [ ] T019 [US2] Implement a client-side fetch call from the chatbot component to the backend's `/chat` endpoint.

---

## Phase 5: Urdu Translation (User Story 3 - P3, Bonus)

**Goal**: Add chapter-level Urdu translation.

**Independent Test**: A language toggle switches content between English and Urdu.

- [ ] T020 [P] [US3] Configure Docusaurus i18n for English and Urdu.
- [ ] T021 [P] [US3] Create Urdu versions of the markdown files.
- [ ] T022 [US3] Generate translated content for one chapter as a proof-of-concept.

---

## Phase 6: Deployment Preparation

**Purpose**: Finalize the project for deployment to GitHub Pages.

- [ ] T023 Update `docusaurus.config.js` with the correct repository name and deployment branch.
- [ ] T024 Create a GitHub Actions workflow file (`.github/workflows/deploy.yml`) to build and deploy the Docusaurus site.
- [ ] T025 Create a `README.md` with instructions on how to run the project locally and deploy it.
