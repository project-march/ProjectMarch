import os

# Retrieve branch name
if os.getenv('CI'):
    branch_name = os.environ['CI_COMMIT_BRANCH']
else:
    import pygit2
    branch_name = pygit2.Repository('.').head.shorthand

extensions = ['sphinx.ext.extlinks',
              'sphinx.ext.todo',
              'sphinx_rtd_theme',
              ]

todo_include_todos = True

# The master toctree document.
master_doc = 'index'

project = 'Project March'
copyright = '2020, Project March'

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = 'Melodic'
# The full version, including alpha/beta/rc tags.
release = 'Melodic'

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'default'

# Name of the style used to generate the html documentation
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'logo_only': True,
    'display_version': False,
    'style_external_links': True,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

html_last_updated_fmt = '%b %d, %Y'
html_show_copyright = True

html_context = {
    'display_gitlab': True,
    'gitlab_user': 'project-march',
    'gitlab_repo': 'tutorials',
    'gitlab_version': branch_name,
    'gitlab_url': 'https://gitlab.com/',
    "conf_py_path": "/",
    "css_files": ['_static/css/override.css'],
    "favicon": "favicon.ico",
    "logo": "logo.png"
}

# Global substitutions
rst_prolog = """
.. |march| replace:: March exoskeleton
"""

# Links
extlinks = {'codedir': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/tutorials/tree/' + html_context['gitlab_version'] + '/doc/%s', ''),
            'rootdir': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/tutorials/tree/' + html_context['gitlab_version'] + '/%s', ''),
            'hardware-interface': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/march/-/tree/main/src/march_hardware_interface/%s', ''),
            'input-device': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/input-device/-/tree/main/%s', ''),
            'march': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/march/-/tree/main/%s', ''),
            'monitor': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/march/-/tree/main/src/march_monitor/%s', ''),
            'simulation': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/march/-/tree/main/src/march_simulation/%s', ''),
            'gait-files': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/march/-/tree/main/src/march_gait_files/%s', ''),
            'ethercat-slaves': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/ethercat-slaves/-/tree/main/%s', ''),
            'gait-generation': (html_context['gitlab_url'] + html_context['gitlab_user'] + '/march/-/tree/main/src/march_rqt_gait_generator/%s', ''),
            'march_website': ('http://projectmarch.nl', '')}

# Output file base name for HTML help builder.
htmlhelp_basename = 'MarchDocumentation'
