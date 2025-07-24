#!/usr/bin/env python3
"""
Auto fix EIDE project paths tool
Fix compilation errors caused by paths containing Chinese characters

Usage:
1. Run in VS Code terminal: python fix_project_paths.py
2. Or add this script to VS Code tasks for automatic execution
"""

import json
import os
import sys
import shutil
import glob
from pathlib import Path

class EIDEProjectFixer:
    def __init__(self, project_root):
        self.project_root = Path(project_root)
        self.eide_config_path = self.project_root / ".eide" / "eide.json"
        
    def get_correct_ti_path(self):
        """
        Auto detect correct TI toolchain path
        """
        # Get current path and try to extract the correct base path
        current_path = str(self.project_root.absolute())
        
        # If current path contains lp_mspm0g3507_mini_examples, extract base path
        if "lp_mspm0g3507_mini_examples" in current_path:
            # Find the position of lp_mspm0g3507_mini_examples
            idx = current_path.find("lp_mspm0g3507_mini_examples")
            # Extract everything before it
            base_path = current_path[:idx].rstrip("\\").rstrip("/")
            
            # Verify the syscfg.bat exists
            syscfg_path = os.path.join(
                base_path,
                "lp_mspm0g3507_mini_examples",
                "lp_mspm0g3507_mini_examples", 
                "tools",
                "keil",
                "syscfg.bat"
            )
            
            if os.path.exists(syscfg_path):
                print(f"Found syscfg.bat at: {syscfg_path}")
                return base_path
        
        # Common TI toolchain installation paths
        possible_paths = [
            "C:/ti",
            "D:/TI", 
            "C:/TI",
            "D:/ti"
        ]
        
        for base_path in possible_paths:
            syscfg_path = os.path.join(
                base_path,
                "lp_mspm0g3507_mini_examples",
                "lp_mspm0g3507_mini_examples", 
                "tools",
                "keil",
                "syscfg.bat"
            )
            if os.path.exists(syscfg_path):
                return base_path
        
        print("Warning: Could not find syscfg.bat in common locations")
        return None
    
    def fix_syscfg_path(self, config_data):
        """
        Fix syscfg tool path
        """
        correct_base_path = self.get_correct_ti_path()
        if not correct_base_path:
            print("Error: Cannot auto detect TI toolchain path")
            return False
            
        # Build correct syscfg.bat path using forward slashes
        correct_syscfg_path = "/".join([
            correct_base_path.replace("\\", "/"), 
            "lp_mspm0g3507_mini_examples", 
            "lp_mspm0g3507_mini_examples", 
            "tools", 
            "keil", 
            "syscfg.bat"
        ])
        
        print(f"Detected correct syscfg path: {correct_syscfg_path}")
        
        # Fix path in beforeBuildTasks
        try:
            # Find builderOptions in the config structure
            builder_options = None
            
            # Check different possible locations
            if "builderOptions" in config_data:
                builder_options = config_data["builderOptions"]
            elif "env" in config_data and "builderOptions" in config_data["env"]:
                builder_options = config_data["env"]["builderOptions"]
            elif "targets" in config_data:
                # Look through all targets
                for target_name, target_config in config_data["targets"].items():
                    if "builderOptions" in target_config:
                        builder_options = target_config["builderOptions"]
                        break
            
            if not builder_options:
                print("Error: Cannot find builderOptions in config")
                return False
                
            fixed_count = 0
            for compiler in builder_options:
                if "beforeBuildTasks" in builder_options[compiler]:
                    for task in builder_options[compiler]["beforeBuildTasks"]:
                        if "syscfg" in task.get("name", "").lower():
                            # Build new command using forward slashes and proper escaping
                            new_command = f'cd ..\\..\\ && "{correct_syscfg_path}" "${{OutDir}}\\..\\.." ncontroller.syscfg'
                            task["command"] = new_command
                            print(f"Fixed syscfg path for {compiler} compiler")
                            fixed_count += 1
            
            if fixed_count > 0:
                print(f"Successfully fixed {fixed_count} syscfg path(s)")
                return True
            else:
                print("No syscfg tasks found to fix")
                return False
                
        except KeyError as e:
            print(f"Config file structure error: {e}")
            return False
    
    def fix_relative_paths(self, config_data):
        """
        Fix other possible relative path issues
        """
        def fix_path_in_dict(obj, key_to_fix="path"):
            if isinstance(obj, dict):
                for key, value in obj.items():
                    if key == key_to_fix and isinstance(value, str):
                        # Ensure correct path separator
                        obj[key] = value.replace("/", "\\")
                    else:
                        fix_path_in_dict(value, key_to_fix)
            elif isinstance(obj, list):
                for item in obj:
                    fix_path_in_dict(item, key_to_fix)
        
        # Fix paths in virtual folders
        fix_path_in_dict(config_data.get("virtualFolder", {}))
        print("Fixed virtual folder paths")
        
    def create_backup(self):
        """
        Create config file backup
        """
        if self.eide_config_path.exists():
            backup_path = self.eide_config_path.with_suffix(".json.backup")
            shutil.copy2(self.eide_config_path, backup_path)
            print(f"Backup created: {backup_path}")
            return True
        return False
    
    def fix_project(self):
        """
        Fix all path issues in project
        """
        print("Starting EIDE project path fix...")
        print(f"Project path: {self.project_root}")
        
        # Check if config file exists
        if not self.eide_config_path.exists():
            print(f"Error: Config file not found {self.eide_config_path}")
            return False
        
        # Create backup
        if not self.create_backup():
            print("Warning: Cannot create backup file")
        
        try:
            # Read config file
            with open(self.eide_config_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # Fix syscfg path
            if not self.fix_syscfg_path(config_data):
                return False
            
            # Fix relative paths
            self.fix_relative_paths(config_data)
            
            # Write back config file
            with open(self.eide_config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2, ensure_ascii=False)
            
            print("Project path fix completed!")
            print("Please try to compile the project again.")
            return True
            
        except json.JSONDecodeError as e:
            print(f"Config file JSON format error: {e}")
            return False
        except Exception as e:
            print(f"Error during fix process: {e}")
            return False

def main():
    """
    Main function
    """
    print("=== EIDE Project Path Auto Fix Tool ===")
    print("This tool fixes compilation errors caused by Chinese character paths\n")
    
    # Get current working directory
    current_dir = os.getcwd()
    
    # Use specified path if command line argument provided
    if len(sys.argv) > 1:
        project_path = sys.argv[1]
    else:
        project_path = current_dir
    
    print(f"Working directory: {project_path}")
    
    # Create fixer instance
    fixer = EIDEProjectFixer(project_path)
    
    # Execute fix
    if fixer.fix_project():
        print("\n? Fix successful!")
        print("Recommended steps:")
        print("1. Restart VS Code")
        print("2. Reopen project")
        print("3. Try to compile project")
    else:
        print("\n? Fix failed!")
        print("Please check error messages and fix manually.")
        sys.exit(1)

if __name__ == "__main__":
    main()
