# Retrieve branch name
branch_name = "main"

extensions = [
    "sphinx.ext.extlinks",
    "sphinx.ext.todo",
    "sphinx_rtd_theme",
]

todo_include_todos = True

# The master toctree document.
master_doc = "index"

project = "Project MARCH"
copyright = "2021, Project MARCH"  # noqa: A001, VNE003

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = "VI"
# The full version, including alpha/beta/rc tags.
release = "VI"

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = "default"

# Name of the style used to generate the html documentation
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "logo_only": True,
    "display_version": False,
    "style_external_links": True,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["static"]

html_last_updated_fmt = "%b %d, %Y"
html_show_copyright = True

html_context = {
    "display_gitlab": False,
    "gitlab_user": "project-march",
    "gitlab_repo": "march",
    "gitlab_version": branch_name,
    "gitlab_url": "https://gitlab.com/",
    "conf_py_path": "/docs",
    "css_files": ["_static/css/override.css"],
    "favicon": "favicon.ico",
    "logo": "logo.png",
}

# Global substitutions
rst_prolog = """
.. |march| replace:: MARCH exoskeleton
"""

gitlab_root = html_context["gitlab_url"] + html_context["gitlab_user"] + "/"

# Links
extlinks = {
    "codedir": (
        gitlab_root + "march/tree/" + html_context["gitlab_version"] + "/doc/%s",
        "",
    ),
    "rootdir": (
        gitlab_root + "march/tree/" + html_context["gitlab_version"] + "/%s",
        "",
    ),
    "hardware-interface": (gitlab_root + ros1_src + "march_hardware_interface/%s", ""),
    "input-device": (gitlab_root + "input-device/-/tree/main/%s", ""),
    "march": (gitlab_root + ros1_src + "%s", ""),
    "monitor": (gitlab_root + ros1_src + "march_monitor/%s", ""),
    "simulation": (gitlab_root + ros1_src + "march_simulation_cpp/%s", ""),
    "gait-files": (gitlab_root + ros1_src + "march_gait_files/%s", ""),
    "ethercat-slaves": (gitlab_root + "ethercat-slaves/-/tree/main/%s", ""),
    "gait-generation": (gitlab_root + ros1_src + "march_rqt_gait_generator/%s", ""),
    "march_website": ("https://projectmarch.nl", ""),
}

# Output file base name for HTML help builder.
htmlhelp_basename = "MarchDocumentation"
