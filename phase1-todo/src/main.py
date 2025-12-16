#!/usr/bin/env python3
"""
Phase I - Todo Console App
A simple in-memory todo application with CRUD operations
"""

class Task:
    """Represents a single todo task"""
    
    def __init__(self, task_id, title, description=""):
        self.id = task_id
        self.title = title
        self.description = description
        self.completed = False
    
    def mark_complete(self):
        """Mark task as completed"""
        self.completed = True
    
    def __str__(self):
        status = "✓" if self.completed else " "
        return f"[{status}] ID: {self.id} | {self.title} | {self.description}"


class TodoApp:
    """Main Todo Application"""
    
    def __init__(self):
        self.tasks = []
        self.next_id = 1
    
    def add_task(self, title, description=""):
        """Add a new task"""
        task = Task(self.next_id, title, description)
        self.tasks.append(task)
        self.next_id += 1
        print(f"✓ Task added: {title}")
    
    def view_tasks(self):
        """Display all tasks"""
        if not self.tasks:
            print("No tasks yet!")
            return
        
        print("\n" + "="*60)
        print("YOUR TASKS")
        print("="*60)
        for task in self.tasks:
            print(task)
        print("="*60 + "\n")
    
    def update_task(self, task_id, new_title=None, new_description=None):
        """Update an existing task"""
        for task in self.tasks:
            if task.id == task_id:
                if new_title:
                    task.title = new_title
                if new_description:
                    task.description = new_description
                print(f"✓ Task {task_id} updated")
                return
        print(f"✗ Task {task_id} not found")
    
    def delete_task(self, task_id):
        """Delete a task"""
        for i, task in enumerate(self.tasks):
            if task.id == task_id:
                self.tasks.pop(i)
                print(f"✓ Task {task_id} deleted")
                return
        print(f"✗ Task {task_id} not found")
    
    def mark_complete(self, task_id):
        """Mark a task as complete"""
        for task in self.tasks:
            if task.id == task_id:
                task.mark_complete()
                print(f"✓ Task {task_id} marked as complete")
                return
        print(f"✗ Task {task_id} not found")
    
    def run(self):
        """Run the main menu loop"""
        print("\n" + "="*60)
        print("WELCOME TO TODO CONSOLE APP - PHASE I")
        print("="*60 + "\n")
        
        while True:
            print("\nMENU:")
            print("1. Add Task")
            print("2. View Tasks")
            print("3. Update Task")
            print("4. Mark Task Complete")
            print("5. Delete Task")
            print("6. Exit")
            print("-" * 60)
            
            choice = input("Enter your choice (1-6): ").strip()
            
            if choice == "1":
                title = input("Enter task title: ").strip()
                description = input("Enter task description (optional): ").strip()
                self.add_task(title, description)
            
            elif choice == "2":
                self.view_tasks()
            
            elif choice == "3":
                try:
                    task_id = int(input("Enter task ID to update: ").strip())
                    new_title = input("Enter new title (or press Enter to skip): ").strip()
                    new_description = input("Enter new description (or press Enter to skip): ").strip()
                    self.update_task(task_id, new_title or None, new_description or None)
                except ValueError:
                    print("✗ Invalid ID")
            
            elif choice == "4":
                try:
                    task_id = int(input("Enter task ID to mark complete: ").strip())
                    self.mark_complete(task_id)
                except ValueError:
                    print("✗ Invalid ID")
            
            elif choice == "5":
                try:
                    task_id = int(input("Enter task ID to delete: ").strip())
                    self.delete_task(task_id)
                except ValueError:
                    print("✗ Invalid ID")
            
            elif choice == "6":
                print("\n✓ Goodbye!\n")
                break
            
            else:
                print("✗ Invalid choice. Please enter 1-6.")


if __name__ == "__main__":
    app = TodoApp()
    app.run()
