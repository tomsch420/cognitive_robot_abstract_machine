pycram.external_interfaces.procthor
===================================

.. py:module:: pycram.external_interfaces.procthor


Classes
-------

.. autoapisummary::

   pycram.external_interfaces.procthor.ProcTHORInterface


Module Contents
---------------

.. py:class:: ProcTHORInterface

   Interface for interacting with the ProcThor environments.
   This class provides methods to scrape, download, and extract .tar.gz files containing ProcThor environments.
   Base URL defaults to 'https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/'


   .. py:attribute:: base_url
      :type:  str
      :value: 'https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/'


      The base URL to scrape for .tar.gz files containing ProcThor environments.



   .. py:attribute:: project_root
      :type:  str

      The root path of the project, used to find resources and other files.



   .. py:method:: get_tarball_links() -> List[str]

      Scrape and return all .tar.gz file links from the specified base URL.

      :return: A list of URLs pointing to .tar.gz files.



   .. py:method:: download_file(url: str, filename: str)

      Download a file from a URL to a destination path.

      :param url: The URL of the file to download.
      :param filename: The name of the file to save as.



   .. py:method:: extract_tar(filename: str, extract_to: str)

      Extract a .tar.gz archive to the specified directory.

      :param filename: The name of the .tar.gz file to extract.
      :param extract_to: Directory where the contents should be extracted.



   .. py:method:: sample_environment(keep_environment: bool = False)

      Fetch and extract a random selection of environments packed in .tar.gz files from a URL.

      :param keep_environment: If True, the environments will be kept in the resources directory, otherwise they will be



