import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="multiobjective",
    version="0.0.1",
    author="Brais González-Rodriguez, Ángel M. González Rueda",
    author_email="braisgonzalez.rodriguez@usc.es, angelmanuel.gonzalez.rueda@usc.es",
    description="Multi-Objective Programming ",
    long_description="Multi-Objective Programming Techniques",
    long_description_content_type="",
    url="https://github.com/braisgonzalezusc/multiobjective.git",
    packages=setuptools.find_packages(),
    # classifiers=[
    #     "Programming Language :: Python :: 3",
    # ],
    #python_requires='>=3.5',
    install_requires=['pyomo', 'numpy'],
)
