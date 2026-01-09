pycram.testing
==============

.. py:module:: pycram.testing


Attributes
----------

.. autoapisummary::

   pycram.testing.logger


Classes
-------

.. autoapisummary::

   pycram.testing.SemanticWorldTestCase
   pycram.testing.EmptyWorldTestCase
   pycram.testing.ApartmentWorldTestCase


Functions
---------

.. autoapisummary::

   pycram.testing.setup_world


Module Contents
---------------

.. py:data:: logger

.. py:function:: setup_world() -> semantic_digital_twin.world.World

.. py:class:: SemanticWorldTestCase(methodName='runTest')

   Bases: :py:obj:`unittest.TestCase`


   A class whose instances are single test cases.

   By default, the test code itself should be placed in a method named
   'runTest'.

   If the fixture may be used for many test cases, create as
   many test methods as are needed. When instantiating such a TestCase
   subclass, specify in the constructor arguments the name of the test method
   that the instance is to execute.

   Test authors should subclass TestCase for their own tests. Construction
   and deconstruction of the test's environment ('fixture') can be
   implemented by overriding the 'setUp' and 'tearDown' methods respectively.

   If it is necessary to override the __init__ method, the base class
   __init__ method must always be called. It is important that subclasses
   should not change the signature of their __init__ method, since instances
   of the classes are instantiated automatically by parts of the framework
   in order to be run.

   When subclassing TestCase, you can set these attributes:
   * failureException: determines which exception will be raised when
       the instance's assertion methods fail; test methods raising this
       exception will be deemed to have 'failed' rather than 'errored'.
   * longMessage: determines whether long messages (including repr of
       objects used in assert methods) will be printed on failure in *addition*
       to any explicit message passed.
   * maxDiff: sets the maximum length of a diff in failure messages
       by assert methods using difflib. It is looked up as an instance
       attribute so can be configured by individual tests if required.


   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World


   .. py:method:: setUpClass()
      :classmethod:


      Hook method for setting up class fixture before running tests in the class.



.. py:class:: EmptyWorldTestCase(methodName='runTest')

   Bases: :py:obj:`unittest.TestCase`


   Base class for unit tests that require and ordinary setup and teardown of the empty bullet-world.


   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World


   .. py:attribute:: render_mode


   .. py:method:: setUpClass()
      :classmethod:


      Hook method for setting up class fixture before running tests in the class.



   .. py:method:: setUp()

      Hook method for setting up the test fixture before exercising it.



   .. py:method:: tearDown()

      Hook method for deconstructing the test fixture after testing it.



.. py:class:: ApartmentWorldTestCase(methodName='runTest')

   Bases: :py:obj:`EmptyWorldTestCase`


   Class for unit tests that require a bullet-world with a PR2, kitchen, milk and cereal.


   .. py:method:: setUpClass()
      :classmethod:


      Hook method for setting up class fixture before running tests in the class.



   .. py:method:: tearDown()

      Hook method for deconstructing the test fixture after testing it.



