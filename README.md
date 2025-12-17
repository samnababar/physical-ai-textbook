# Physical AI & Humanoid Robotics Textbook

This repository contains an AI-native technical textbook for "Physical AI & Humanoid Robotics," built using Docusaurus. It also includes a Retrieval-Augmented Generation (RAG) chatbot capable of answering questions based on the book's content.

## Author

This project and book are created by **Abdul Rafay**.

## Project Structure

-   `textbook/`: The Docusaurus project for the textbook.
-   `api/`: A FastAPI application for the RAG chatbot's backend (Vercel Serverless Function).

## Getting Started

### Prerequisites

-   Node.js (version 18.x or later)
-   npm (comes with Node.js)
-   Python (version 3.11 or later)
-   pip (comes with Python)

### 1. Clone the repository

```bash
git clone https://github.com/samnababar/Textbook.git
cd Textbook
```

### 2. Set up the Docusaurus Frontend (Textbook)

Navigate to the `textbook` directory and install dependencies:

```bash
cd textbook
npm install
```

To run the Docusaurus development server locally:

```bash
npm start
```

This will open the textbook in your browser at `http://localhost:3000`.

### 3. Set up the FastAPI Backend (RAG Chatbot)

Open a new terminal at the root of the project.

Create a Python virtual environment and install dependencies from the root `requirements.txt`:

```bash
python -m venv venv
./venv/Scripts/activate # On Windows
# source venv/bin/activate # On macOS/Linux
pip install -r requirements.txt
```

To run the FastAPI backend locally, you can use the Vercel CLI:

```bash
# Install vercel cli if you haven't already
npm install -g vercel
# Run the development server
vercel dev
```

The Vercel development server will run both your Docusaurus app and the Python API. The textbook will be available at `http://localhost:3000` and the API at `http://localhost:3000/api/chat`.

## Deployment to Vercel

This project is configured for easy deployment to Vercel.

1.  **Sign up for a Vercel account:**
    *   Go to [https://vercel.com/signup](https://vercel.com/signup) and create an account. It's recommended to sign up with your GitHub account.

2.  **Import your Project:**
    *   From your Vercel dashboard, click on "Add New..." -> "Project".
    *   Under "Import Git Repository," select your `Textbook` repository from GitHub.

3.  **Configure and Deploy:**
    *   Vercel should automatically detect the monorepo setup from the `vercel.json` file.
    *   You should not need to change any settings.
    *   Click "Deploy".

Vercel will build and deploy both your Docusaurus site and the Python API, and provide you with a live URL.