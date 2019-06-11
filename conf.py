import sys, os

# Retrieve branch name
try:
    travis = os.environ['TRAVIS']
    branch_name = os.environ['BRANCH']
except KeyError:  # Local
    import pygit2
    branch_name = pygit2.Repository('.').head.shorthand

sys.path += [ os.path.abspath( '_scripts' )]

extensions = ['sphinx.ext.extlinks', 'sphinx.ext.githubpages', 'tutorialformatter']

# The master toctree document.
master_doc = 'index'

# The suffix of source filenames.
source_suffix = '.rst'

project = u'march_tutorials'

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = 'Kinetic'
# The full version, including alpha/beta/rc tags.
release = 'Kinetic'

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# Name of the style used to generate the html documentation
html_theme = 'sphinx_rtd_theme'
html_theme_path = ['_themes',]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

html_context = {
    "display_github": True,
    "github_user": "project-march",
    "github_repo": "tutorials",
    "github_version": branch_name,
    "conf_py_path": "",
    "source_suffix": source_suffix,
    "css_files": ['_static/override.css'],
    "favicon": "favicon.ico"
#  "logo": "logo.png"
}

# Links
ros_distro = 'kinetic'
extlinks = {'codedir': ('https://github.com/' + html_context["github_user"] + '/tutorials/tree/' + html_context["github_version"] + '/doc/%s', ''),
            'march_website': ('http://projectmarch.nl', '')}

# Output file base name for HTML help builder.
htmlhelp_basename = 'MarchDocumentation'
