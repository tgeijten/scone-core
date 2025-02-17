import inspect
import re
import typing
from types import MethodType, FunctionType, BuiltinFunctionType
from typing import Any, Callable
import re

def remove_self_argument(signature: str) -> str:
    signature = re.sub(r'\bself\s*:\s*[^,)]*,?\s*', '', signature, count=1)
    return re.sub(r'\(,\s*', '(', signature)

def emphasize_name(signature: str) -> str:
    return re.sub(r'^(\w+)', r'**\1**', signature)

# def remove_self_argument(signature: str) -> str:
#     # Regular expression to match 'self: <type>' at the beginning of the argument list
#     pattern = r'\bself\s*:\s*[^,)]*,?\s*'
    
#     # Replace the matched pattern with an empty string
#     new_signature = re.sub(pattern, '', signature, count=1)
    
#     # Ensure proper formatting (remove extra comma if present)
#     new_signature = re.sub(r'\(,\s*', '(', new_signature)
    
#     return new_signature


def make_table_row(doc):
    doc = doc.strip().split('\n')
    funs = remove_self_argument(doc[0]).split(" -> ")
    fun = funs[0]
    fun = fun.replace("'", "%%'%%")
    fun = emphasize_name(fun)
    ret = funs[1]
    info = doc[2] if (len(doc) > 2) else ""
    info = info.replace("^", "%%^%%")
    return f"| {fun} | {ret} | {info} |"

def generate_dokuwiki_docs(module):
    """
    Generate DokuWiki formatted documentation for a Python module using a two-column table layout.
    Column 1: Function signature (name, arguments, and return type)
    Column 2: Description
    
    Args:
        module: The module to document
    
    Returns:
        str: DokuWiki formatted documentation
    """
    output = []
    output.append('====== ScopePy Reference Manual ======\n')
    
    # Get all classes
    classes = inspect.getmembers(module, inspect.isclass)
    if classes:
        output.append('===== Classes =====\n')
        for class_name, class_obj in classes:
            if class_obj.__module__ == module.__name__:
                output.append(f'  * [[#{class_name}|{class_name}]]')
        output.append('\n')
    
    # Get module functions
    functions = []
    for name in dir(module):
        if name.startswith('__'):
            continue
            
        attr = getattr(module, name)
        if isinstance(attr, (FunctionType, BuiltinFunctionType)):
            functions.append((name, attr))
    
    # Add functions section with table
    if functions:
        output.append('===== Functions =====\n')
        # Add table header
        output.append('^ Signature ^ Returns ^ Description ^')
        
        for func_name, func_obj in sorted(functions):
            try:
                signature = str(inspect.signature(func_obj))
                
                # Get return type annotation
                return_type = ""
                if hasattr(func_obj, '__annotations__') and 'return' in func_obj.__annotations__:
                    return_annotation = func_obj.__annotations__['return']
                    if hasattr(return_annotation, '__name__'):
                        return_type = f" -> {return_annotation.__name__}"
                    else:
                        return_type = f" -> {str(return_annotation)}"
            except (ValueError, TypeError, AttributeError):
                signature = '()'
                return_type = ""
            
            # Get docstring for description only
            row = ""
            if func_obj.__doc__:
                row = make_table_row(func_obj.__doc__)

            # First column: function name, signature, and return type
            function_spec = f"{func_name}{signature}{return_type}"
            
            # Add function to table
            # output.append(f'| {function}  | {info}  |')
            output.append(row)
        
        output.append('')  # Add spacing after functions table
    
    # Add classes section
    # output.append('====== Classes ======\n')
    
    # Document each class
    for class_name, class_obj in classes:
        if class_obj.__module__ == module.__name__:
            output.append(f'===== {class_name} =====\n')
            
            # Add class description if available
            if class_obj.__doc__:
                output.append(class_obj.__doc__.strip() + '\n')
            
            # Get all methods and properties
            methods = []
            properties = []
            
            # Get members from the class
            for name, member in class_obj.__dict__.items():
                if name.startswith('__'):
                    continue
                
                if isinstance(member, property):
                    properties.append((name, member))
                elif isinstance(member, (staticmethod, classmethod)):
                    methods.append((name, member.__get__(None, class_obj)))
                elif inspect.isfunction(member) or isinstance(member, MethodType):
                    methods.append((name, member))
                elif callable(member):
                    methods.append((name, member))
            
            # Process methods with table
            if methods:
                # output.append('=== Methods ===\n')
                output.append('^ Signature ^ Returns ^ Description ^')
                
                for method_name, method_obj in sorted(methods):
                    try:
                        if isinstance(method_obj, MethodType):
                            method_obj = method_obj.__func__
                        signature = str(inspect.signature(method_obj))
                        # Remove self/cls parameter
                        signature = signature.replace('(self, ', '(').replace('(self)', '()')
                        signature = signature.replace('(cls, ', '(').replace('(cls)', '()')
                        
                        # Get return type
                        return_type = ""
                        if hasattr(method_obj, '__annotations__') and 'return' in method_obj.__annotations__:
                            return_annotation = method_obj.__annotations__['return']
                            if hasattr(return_annotation, '__name__'):
                                return_type = f" -> {return_annotation.__name__}"
                            else:
                                return_type = f" -> {str(return_annotation)}"
                    except (ValueError, TypeError, AttributeError):
                        signature = '()'
                        return_type = ""
                    
                    # Get method description
                    info = ""
                    description = ""
                    if method_obj.__doc__:
                        row = make_table_row(method_obj.__doc__)

                    # First column: method name, signature, and return type
                    method_spec = f"{method_name}{signature}{return_type}"
                    
                    # Add method to table
                    output.append(row)
                
                output.append('')  # Add spacing after methods table
            
            # Process properties with table
            if properties:
                output.append('=== Properties ===\n')
                output.append('^ Signature  ^ Description  ^')
                
                for prop_name, prop_obj in sorted(properties):
                    # Get property type if available
                    prop_type = ""
                    if hasattr(prop_obj, '__annotations__') and 'return' in prop_obj.__annotations__:
                        return_annotation = prop_obj.__annotations__['return']
                        if hasattr(return_annotation, '__name__'):
                            prop_type = f" -> {return_annotation.__name__}"
                        else:
                            prop_type = f" -> {str(return_annotation)}"
                    
                    # Get property description
                    description = ""
                    if prop_obj.__doc__:
                        description = prop_obj.__doc__.strip()
                        description = description.split('\n')[0]  # Take only the first line
                        description = re.sub(r'\s+', ' ', description)
                    
                    # First column: property name and type
                    prop_spec = f"{prop_name}{prop_type}"
                    
                    # Add property to table
                    output.append(f"| ''{prop_spec}'' | {description} |")
                
                output.append('')  # Add spacing after properties table
            
            output.append('')  # Add spacing between classes
    
    text = '\n'.join(output)
    for c, obj in classes:
        text = text.replace(f"sconepy.{c}", f"[[#{c}|{c}]]")
    text = text.replace("[[[", "[ [[")
    text = text.replace("]]]", "]] ]")
    return text

# Example usage:
if __name__ == '__main__':
    try:
        from sconetoolsdev import sconepy
        print("Generating documentation...")
        wiki_docs = generate_dokuwiki_docs(sconepy)
        
        # Write to file
        with open('../sconepy/sconepy.txt', 'w') as f:
            f.write(wiki_docs)
            
        print("Documentation has been generated in 'sconepy_docs.txt'")
        
    except ImportError:
        print("Could not import sconepy module. Please ensure it is installed.")