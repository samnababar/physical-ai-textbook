# Phase I – Todo Console App

## Description
A Python console-based Todo application built using spec-driven development. This is Phase I of Hackathon II - a simple, focused implementation with in-memory storage and a menu-driven interface.

## Features
- ✓ Add task with title and description
- ✓ View all tasks with completion status
- ✓ Update existing tasks
- ✓ Delete tasks
- ✓ Mark tasks as complete
- ✓ Console-based menu interface

## Task Model
- **id**: integer (auto-incrementing)
- **title**: string
- **description**: string
- **completed**: boolean

## Architecture
- **Storage**: In-memory list (no database)
- **Interface**: Console menu-driven
- **Language**: Python 3
- **Dependencies**: None (pure Python)

## How to Run

```bash
python src/main.py
```

## Usage
1. Launch the app
2. Choose from the menu options (1-6)
3. Follow prompts to add, view, update, or delete tasks
4. Select option 6 to exit

## Project Structure
```
phase1-todo/
├── src/
│   └── main.py          # Main application
├── specs/
│   └── task-crud.md     # Feature specification
├── constitution.md       # Project guidelines
├── CLAUDE.md            # AI instructions
└── README.md            # This file
```

## Development Approach
- **Spec-Driven Development**: All features defined in specs/task-crud.md
- **Simplicity First**: No databases, no web framework, no extra features
- **Clean Code**: Well-organized, readable, and maintainable

## Phase I Status
✓ COMPLETE - All required features implemented and tested
