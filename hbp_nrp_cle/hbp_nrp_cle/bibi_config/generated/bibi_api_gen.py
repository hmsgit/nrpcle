# ./bibi_api_gen.py
# -*- coding: utf-8 -*-
# PyXB bindings for NM:d8bc1000793d9bf647fd97a1512e459e4ce45e64
# Generated 2015-09-11 16:15:12.712473 by PyXB version 1.2.4 using Python 2.7.6.final.0
# Namespace http://schemas.humanbrainproject.eu/SP10/2014/BIBI

from __future__ import unicode_literals
import pyxb
import pyxb.binding
import pyxb.binding.saxer
import io
import pyxb.utils.utility
import pyxb.utils.domutils
import sys
import pyxb.utils.six as _six

# Unique identifier for bindings created at the same time
_GenerationUID = pyxb.utils.utility.UniqueIdentifier('urn:uuid:8430d416-588f-11e5-95c8-b8ac6f46110b')

# Version of PyXB used to generate the bindings
_PyXBVersion = '1.2.4'
# Generated bindings are not compatible across PyXB versions
if pyxb.__version__ != _PyXBVersion:
    raise pyxb.PyXBVersionError(_PyXBVersion)

# Import bindings for namespaces imported into schema
import pyxb.binding.datatypes

# todo: remove hack: check if namespace already exists
ns = pyxb.namespace.NamespaceForURI('http://schemas.humanbrainproject.eu/SP10/2014/BIBI',
                                      create_if_missing=False)

if ns is not None:
    print "Namespace: for bibi already exists."
    from hbp_nrp_commons.generated.bibi_api_gen import *
    pass
else:

    # NOTE: All namespace declarations are reserved within the binding
    Namespace = pyxb.namespace.NamespaceForURI('http://schemas.humanbrainproject.eu/SP10/2014/BIBI', create_if_missing=True)
    Namespace.configureCategories(['typeBinding', 'elementBinding'])

    def CreateFromDocument (xml_text, default_namespace=None, location_base=None):
        """Parse the given XML and use the document element to create a
        Python instance.

        @param xml_text An XML document.  This should be data (Python 2
        str or Python 3 bytes), or a text (Python 2 unicode or Python 3
        str) in the L{pyxb._InputEncoding} encoding.

        @keyword default_namespace The L{pyxb.Namespace} instance to use as the
        default namespace where there is no default namespace in scope.
        If unspecified or C{None}, the namespace of the module containing
        this function will be used.

        @keyword location_base: An object to be recorded as the base of all
        L{pyxb.utils.utility.Location} instances associated with events and
        objects handled by the parser.  You might pass the URI from which
        the document was obtained.
        """

        if pyxb.XMLStyle_saxer != pyxb._XMLStyle:
            dom = pyxb.utils.domutils.StringToDOM(xml_text)
            return CreateFromDOM(dom.documentElement, default_namespace=default_namespace)
        if default_namespace is None:
            default_namespace = Namespace.fallbackNamespace()
        saxer = pyxb.binding.saxer.make_parser(fallback_namespace=default_namespace, location_base=location_base)
        handler = saxer.getContentHandler()
        xmld = xml_text
        if isinstance(xmld, _six.text_type):
            xmld = xmld.encode(pyxb._InputEncoding)
        saxer.parse(io.BytesIO(xmld))
        instance = handler.rootObject()
        return instance

    def CreateFromDOM (node, default_namespace=None):
        """Create a Python instance from the given DOM node.
        The node tag must correspond to an element declaration in this module.

        @deprecated: Forcing use of DOM interface is unnecessary; use L{CreateFromDocument}."""
        if default_namespace is None:
            default_namespace = Namespace.fallbackNamespace()
        return pyxb.binding.basis.element.AnyCreateFromDOM(node, default_namespace)


    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SDF_Filename
    class SDF_Filename (pyxb.binding.datatypes.string):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SDF_Filename')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 36, 4)
        _Documentation = None
    SDF_Filename._CF_pattern = pyxb.binding.facets.CF_pattern()
    SDF_Filename._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._/]*\\.sdf')
    SDF_Filename._InitializeFacetMap(SDF_Filename._CF_pattern)
    Namespace.addCategoryObject('typeBinding', 'SDF_Filename', SDF_Filename)

    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}H5_Filename
    class H5_Filename (pyxb.binding.datatypes.string):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'H5_Filename')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 42, 4)
        _Documentation = None
    H5_Filename._CF_pattern = pyxb.binding.facets.CF_pattern()
    H5_Filename._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._/]*\\.h5')
    H5_Filename._InitializeFacetMap(H5_Filename._CF_pattern)
    Namespace.addCategoryObject('typeBinding', 'H5_Filename', H5_Filename)

    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Python_Filename
    class Python_Filename (pyxb.binding.datatypes.string):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Python_Filename')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 48, 4)
        _Documentation = None
    Python_Filename._CF_pattern = pyxb.binding.facets.CF_pattern()
    Python_Filename._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._/]*\\.py')
    Python_Filename._InitializeFacetMap(Python_Filename._CF_pattern)
    Namespace.addCategoryObject('typeBinding', 'Python_Filename', Python_Filename)

    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Script_Filename
    class Script_Filename (pyxb.binding.datatypes.string):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Script_Filename')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 54, 4)
        _Documentation = None
    Script_Filename._CF_pattern = pyxb.binding.facets.CF_pattern()
    Script_Filename._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._/]*\\.sh')
    Script_Filename._InitializeFacetMap(Script_Filename._CF_pattern)
    Namespace.addCategoryObject('typeBinding', 'Script_Filename', Script_Filename)

    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronTarget
    class NeuronTarget (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronTarget')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 124, 4)
        _Documentation = None
    NeuronTarget._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=NeuronTarget, enum_prefix=None)
    NeuronTarget.Inhibitory = NeuronTarget._CF_enumeration.addEnumeration(unicode_value='Inhibitory', tag='Inhibitory')
    NeuronTarget.Excitatory = NeuronTarget._CF_enumeration.addEnumeration(unicode_value='Excitatory', tag='Excitatory')
    NeuronTarget._InitializeFacetMap(NeuronTarget._CF_enumeration)
    Namespace.addCategoryObject('typeBinding', 'NeuronTarget', NeuronTarget)

    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TemplatePattern
    class TemplatePattern (pyxb.binding.datatypes.string):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TemplatePattern')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 304, 4)
        _Documentation = None
    TemplatePattern._CF_pattern = pyxb.binding.facets.CF_pattern()
    TemplatePattern._CF_pattern.addPattern(pattern='(\\(\\s*)*(i|\\d+)(\\s*(\\+|\\*)\\s*(\\(\\s*)*(i|\\d+)\\s*|\\))*')
    TemplatePattern._InitializeFacetMap(TemplatePattern._CF_pattern)
    Namespace.addCategoryObject('typeBinding', 'TemplatePattern', TemplatePattern)

    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceType
    class DeviceType (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'DeviceType')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 341, 4)
        _Documentation = None
    DeviceType._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=DeviceType, enum_prefix=None)
    DeviceType.ACSource = DeviceType._CF_enumeration.addEnumeration(unicode_value='ACSource', tag='ACSource')
    DeviceType.DCSource = DeviceType._CF_enumeration.addEnumeration(unicode_value='DCSource', tag='DCSource')
    DeviceType.FixedFrequency = DeviceType._CF_enumeration.addEnumeration(unicode_value='FixedFrequency', tag='FixedFrequency')
    DeviceType.LeakyIntegratorAlpha = DeviceType._CF_enumeration.addEnumeration(unicode_value='LeakyIntegratorAlpha', tag='LeakyIntegratorAlpha')
    DeviceType.LeakyIntegratorExp = DeviceType._CF_enumeration.addEnumeration(unicode_value='LeakyIntegratorExp', tag='LeakyIntegratorExp')
    DeviceType.NCSource = DeviceType._CF_enumeration.addEnumeration(unicode_value='NCSource', tag='NCSource')
    DeviceType.Poisson = DeviceType._CF_enumeration.addEnumeration(unicode_value='Poisson', tag='Poisson')
    DeviceType.SpikeRecorder = DeviceType._CF_enumeration.addEnumeration(unicode_value='SpikeRecorder', tag='SpikeRecorder')
    DeviceType.PopulationRate = DeviceType._CF_enumeration.addEnumeration(unicode_value='PopulationRate', tag='PopulationRate')
    DeviceType._InitializeFacetMap(DeviceType._CF_enumeration)
    Namespace.addCategoryObject('typeBinding', 'DeviceType', DeviceType)

    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}RobotTopicAddress
    class RobotTopicAddress (pyxb.binding.datatypes.string):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RobotTopicAddress')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 364, 4)
        _Documentation = None
    RobotTopicAddress._CF_pattern = pyxb.binding.facets.CF_pattern()
    RobotTopicAddress._CF_pattern.addPattern(pattern='(/[a-zA-Z0-9_-]+)+(/)?')
    RobotTopicAddress._InitializeFacetMap(RobotTopicAddress._CF_pattern)
    Namespace.addCategoryObject('typeBinding', 'RobotTopicAddress', RobotTopicAddress)

    # Union simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Brain_Filename
    # superclasses pyxb.binding.datatypes.anySimpleType
    class Brain_Filename (pyxb.binding.basis.STD_union):

        """Simple type that is a union of H5_Filename, Python_Filename."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Brain_Filename')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 32, 4)
        _Documentation = None

        _MemberTypes = ( H5_Filename, Python_Filename, )
    Brain_Filename._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=Brain_Filename)
    Brain_Filename._CF_pattern = pyxb.binding.facets.CF_pattern()
    Brain_Filename._InitializeFacetMap(Brain_Filename._CF_enumeration,
       Brain_Filename._CF_pattern)
    Namespace.addCategoryObject('typeBinding', 'Brain_Filename', Brain_Filename)

    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBIConfiguration with content type ELEMENT_ONLY
    class BIBIConfiguration (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBIConfiguration with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BIBIConfiguration')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 9, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}brainModel uses Python identifier brainModel
        __brainModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'brainModel'), 'brainModel', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIbrainModel', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 11, 12), )


        brainModel = property(__brainModel.value, __brainModel.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}bodyModel uses Python identifier bodyModel
        __bodyModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'bodyModel'), 'bodyModel', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIbodyModel', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 12, 12), )


        bodyModel = property(__bodyModel.value, __bodyModel.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}extRobotController uses Python identifier extRobotController
        __extRobotController = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'extRobotController'), 'extRobotController', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIextRobotController', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 13, 12), )


        extRobotController = property(__extRobotController.value, __extRobotController.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectors uses Python identifier connectors
        __connectors = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectors'), 'connectors', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIconnectors', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 15, 12), )


        connectors = property(__connectors.value, __connectors.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamics uses Python identifier synapseDynamics
        __synapseDynamics = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), 'synapseDynamics', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamics', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 17, 12), )


        synapseDynamics = property(__synapseDynamics.value, __synapseDynamics.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}transferFunction uses Python identifier transferFunction
        __transferFunction = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'transferFunction'), 'transferFunction', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBItransferFunction', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 19, 12), )


        transferFunction = property(__transferFunction.value, __transferFunction.set, None, None)

        _ElementMap.update({
            __brainModel.name() : __brainModel,
            __bodyModel.name() : __bodyModel,
            __extRobotController.name() : __extRobotController,
            __connectors.name() : __connectors,
            __synapseDynamics.name() : __synapseDynamics,
            __transferFunction.name() : __transferFunction
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'BIBIConfiguration', BIBIConfiguration)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BrainModel with content type ELEMENT_ONLY
    class BrainModel (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BrainModel with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BrainModel')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 24, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}file uses Python identifier file
        __file = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'file'), 'file', '__httpschemas_humanbrainproject_euSP102014BIBI_BrainModel_httpschemas_humanbrainproject_euSP102014BIBIfile', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 26, 12), )


        file = property(__file.value, __file.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}populations uses Python identifier populations
        __populations = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'populations'), 'populations', '__httpschemas_humanbrainproject_euSP102014BIBI_BrainModel_httpschemas_humanbrainproject_euSP102014BIBIpopulations', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 27, 12), )


        populations = property(__populations.value, __populations.set, None, None)

        _ElementMap.update({
            __file.name() : __file,
            __populations.name() : __populations
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'BrainModel', BrainModel)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TransferFunction with content type EMPTY
    class TransferFunction (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TransferFunction with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TransferFunction')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 60, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'TransferFunction', TransferFunction)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector with content type EMPTY
    class NeuronConnector (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronConnector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 131, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute weights uses Python identifier weights
        __weights = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'weights'), 'weights', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnector_weights', pyxb.binding.datatypes.double)
        __weights._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 132, 8)
        __weights._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 132, 8)

        weights = property(__weights.value, __weights.set, None, None)


        # Attribute delays uses Python identifier delays
        __delays = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'delays'), 'delays', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnector_delays', pyxb.binding.datatypes.double)
        __delays._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 133, 8)
        __delays._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 133, 8)

        delays = property(__delays.value, __delays.set, None, None)


        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnector_name', pyxb.binding.datatypes.string, unicode_default='default')
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 134, 8)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 134, 8)

        name = property(__name.value, __name.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __weights.name() : __weights,
            __delays.name() : __delays,
            __name.name() : __name
        })
    Namespace.addCategoryObject('typeBinding', 'NeuronConnector', NeuronConnector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnectorRef with content type EMPTY
    class NeuronConnectorRef (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnectorRef with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronConnectorRef')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 137, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute ref uses Python identifier ref
        __ref = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'ref'), 'ref', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnectorRef_ref', pyxb.binding.datatypes.string, required=True)
        __ref._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 140, 16)
        __ref._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 140, 16)

        ref = property(__ref.value, __ref.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __ref.name() : __ref
        })
    Namespace.addCategoryObject('typeBinding', 'NeuronConnectorRef', NeuronConnectorRef)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamics with content type EMPTY
    class SynapseDynamics (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamics with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SynapseDynamics')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 165, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_SynapseDynamics_name', pyxb.binding.datatypes.string, unicode_default='default')
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 166, 8)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 166, 8)

        name = property(__name.value, __name.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __name.name() : __name
        })
    Namespace.addCategoryObject('typeBinding', 'SynapseDynamics', SynapseDynamics)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamicsRef with content type EMPTY
    class SynapseDynamicsRef (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamicsRef with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SynapseDynamicsRef')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 169, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute ref uses Python identifier ref
        __ref = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'ref'), 'ref', '__httpschemas_humanbrainproject_euSP102014BIBI_SynapseDynamicsRef_ref', pyxb.binding.datatypes.string, required=True)
        __ref._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 172, 16)
        __ref._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 172, 16)

        ref = property(__ref.value, __ref.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __ref.name() : __ref
        })
    Namespace.addCategoryObject('typeBinding', 'SynapseDynamicsRef', SynapseDynamicsRef)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronGroupSelector with content type EMPTY
    class NeuronGroupSelector (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronGroupSelector with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronGroupSelector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 229, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'NeuronGroupSelector', NeuronGroupSelector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector with content type EMPTY
    class NeuronSelector (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronSelector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 258, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute population uses Python identifier population
        __population = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'population'), 'population', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronSelector_population', pyxb.binding.datatypes.string)
        __population._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 259, 8)
        __population._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 259, 8)

        population = property(__population.value, __population.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __population.name() : __population
        })
    Namespace.addCategoryObject('typeBinding', 'NeuronSelector', NeuronSelector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelectorTemplate with content type EMPTY
    class NeuronSelectorTemplate (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelectorTemplate with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronSelectorTemplate')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 310, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'NeuronSelectorTemplate', NeuronSelectorTemplate)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}FlowExpression with content type EMPTY
    class FlowExpression (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}FlowExpression with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'FlowExpression')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 370, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'FlowExpression', FlowExpression)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Argument with content type ELEMENT_ONLY
    class Argument (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Argument with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Argument')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 442, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}value uses Python identifier value_
        __value = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'value'), 'value_', '__httpschemas_humanbrainproject_euSP102014BIBI_Argument_httpschemas_humanbrainproject_euSP102014BIBIvalue', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 444, 12), )


        value_ = property(__value.value, __value.set, None, None)


        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_Argument_name', pyxb.binding.datatypes.string, required=True)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 446, 8)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 446, 8)

        name = property(__name.value, __name.set, None, None)

        _ElementMap.update({
            __value.name() : __value
        })
        _AttributeMap.update({
            __name.name() : __name
        })
    Namespace.addCategoryObject('typeBinding', 'Argument', Argument)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Local with content type ELEMENT_ONLY
    class Local (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Local with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Local')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 449, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
        __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_Local_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 451, 12), )


        body = property(__body.value, __body.set, None, None)


        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_Local_name', pyxb.binding.datatypes.string, required=True)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 453, 8)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 453, 8)

        name = property(__name.value, __name.set, None, None)

        _ElementMap.update({
            __body.name() : __body
        })
        _AttributeMap.update({
            __name.name() : __name
        })
    Namespace.addCategoryObject('typeBinding', 'Local', Local)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}PythonTransferFunction with content type MIXED
    class PythonTransferFunction (TransferFunction):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}PythonTransferFunction with content type MIXED"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_MIXED
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'PythonTransferFunction')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 63, 4)
        _ElementMap = TransferFunction._ElementMap.copy()
        _AttributeMap = TransferFunction._AttributeMap.copy()
        # Base type is TransferFunction

        # Attribute src uses Python identifier src
        __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014BIBI_PythonTransferFunction_src', Python_Filename)
        __src._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 66, 16)
        __src._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 66, 16)

        src = property(__src.value, __src.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __src.name() : __src
        })
    Namespace.addCategoryObject('typeBinding', 'PythonTransferFunction', PythonTransferFunction)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction with content type ELEMENT_ONLY
    class BIBITransferFunction (TransferFunction):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BIBITransferFunction')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 72, 4)
        _ElementMap = TransferFunction._ElementMap.copy()
        _AttributeMap = TransferFunction._AttributeMap.copy()
        # Base type is TransferFunction

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local uses Python identifier local
        __local = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'local'), 'local', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBIlocal', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20), )


        local = property(__local.value, __local.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device uses Python identifier device
        __device = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'device'), 'device', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBIdevice', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 78, 24), )


        device = property(__device.value, __device.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}deviceGroup uses Python identifier deviceGroup
        __deviceGroup = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup'), 'deviceGroup', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBIdeviceGroup', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 79, 24), )


        deviceGroup = property(__deviceGroup.value, __deviceGroup.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}topic uses Python identifier topic
        __topic = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'topic'), 'topic', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBItopic', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20), )


        topic = property(__topic.value, __topic.set, None, None)


        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_name', pyxb.binding.datatypes.string)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 84, 16)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 84, 16)

        name = property(__name.value, __name.set, None, None)

        _ElementMap.update({
            __local.name() : __local,
            __device.name() : __device,
            __deviceGroup.name() : __deviceGroup,
            __topic.name() : __topic
        })
        _AttributeMap.update({
            __name.name() : __name
        })
    Namespace.addCategoryObject('typeBinding', 'BIBITransferFunction', BIBITransferFunction)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}OneToOneConnector with content type EMPTY
    class OneToOneConnector (NeuronConnector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}OneToOneConnector with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'OneToOneConnector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 145, 4)
        _ElementMap = NeuronConnector._ElementMap.copy()
        _AttributeMap = NeuronConnector._AttributeMap.copy()
        # Base type is NeuronConnector

        # Attribute weights inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector

        # Attribute delays inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector

        # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'OneToOneConnector', OneToOneConnector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}AllToAllConnector with content type EMPTY
    class AllToAllConnector (NeuronConnector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}AllToAllConnector with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'AllToAllConnector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 151, 4)
        _ElementMap = NeuronConnector._ElementMap.copy()
        _AttributeMap = NeuronConnector._AttributeMap.copy()
        # Base type is NeuronConnector

        # Attribute weights inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector

        # Attribute delays inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector

        # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'AllToAllConnector', AllToAllConnector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}FixedNumberPreConnector with content type EMPTY
    class FixedNumberPreConnector (NeuronConnector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}FixedNumberPreConnector with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'FixedNumberPreConnector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 157, 4)
        _ElementMap = NeuronConnector._ElementMap.copy()
        _AttributeMap = NeuronConnector._AttributeMap.copy()
        # Base type is NeuronConnector

        # Attribute weights inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector

        # Attribute delays inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector

        # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector

        # Attribute count uses Python identifier count
        __count = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'count'), 'count', '__httpschemas_humanbrainproject_euSP102014BIBI_FixedNumberPreConnector_count', pyxb.binding.datatypes.positiveInteger)
        __count._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 160, 16)
        __count._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 160, 16)

        count = property(__count.value, __count.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __count.name() : __count
        })
    Namespace.addCategoryObject('typeBinding', 'FixedNumberPreConnector', FixedNumberPreConnector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TsodyksMarkramMechanism with content type EMPTY
    class TsodyksMarkramMechanism (SynapseDynamics):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TsodyksMarkramMechanism with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TsodyksMarkramMechanism')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 177, 4)
        _ElementMap = SynapseDynamics._ElementMap.copy()
        _AttributeMap = SynapseDynamics._AttributeMap.copy()
        # Base type is SynapseDynamics

        # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamics

        # Attribute u uses Python identifier u
        __u = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'u'), 'u', '__httpschemas_humanbrainproject_euSP102014BIBI_TsodyksMarkramMechanism_u', pyxb.binding.datatypes.double)
        __u._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 180, 16)
        __u._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 180, 16)

        u = property(__u.value, __u.set, None, None)


        # Attribute tau_rec uses Python identifier tau_rec
        __tau_rec = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'tau_rec'), 'tau_rec', '__httpschemas_humanbrainproject_euSP102014BIBI_TsodyksMarkramMechanism_tau_rec', pyxb.binding.datatypes.double)
        __tau_rec._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 181, 16)
        __tau_rec._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 181, 16)

        tau_rec = property(__tau_rec.value, __tau_rec.set, None, None)


        # Attribute tau_facil uses Python identifier tau_facil
        __tau_facil = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'tau_facil'), 'tau_facil', '__httpschemas_humanbrainproject_euSP102014BIBI_TsodyksMarkramMechanism_tau_facil', pyxb.binding.datatypes.double)
        __tau_facil._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 182, 16)
        __tau_facil._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 182, 16)

        tau_facil = property(__tau_facil.value, __tau_facil.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __u.name() : __u,
            __tau_rec.name() : __tau_rec,
            __tau_facil.name() : __tau_facil
        })
    Namespace.addCategoryObject('typeBinding', 'TsodyksMarkramMechanism', TsodyksMarkramMechanism)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceChannel with content type ELEMENT_ONLY
    class DeviceChannel (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceChannel with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'DeviceChannel')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 187, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}neurons uses Python identifier neurons
        __neurons = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'neurons'), 'neurons', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIneurons', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 189, 12), )


        neurons = property(__neurons.value, __neurons.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connector uses Python identifier connector
        __connector = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connector'), 'connector', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIconnector', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 191, 16), )


        connector = property(__connector.value, __connector.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectorRef uses Python identifier connectorRef
        __connectorRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), 'connectorRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIconnectorRef', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 192, 16), )


        connectorRef = property(__connectorRef.value, __connectorRef.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamics uses Python identifier synapseDynamics
        __synapseDynamics = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), 'synapseDynamics', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamics', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 196, 16), )


        synapseDynamics = property(__synapseDynamics.value, __synapseDynamics.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamicsRef uses Python identifier synapseDynamicsRef
        __synapseDynamicsRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), 'synapseDynamicsRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamicsRef', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 198, 16), )


        synapseDynamicsRef = property(__synapseDynamicsRef.value, __synapseDynamicsRef.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}target uses Python identifier target
        __target = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'target'), 'target', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBItarget', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 201, 12), )


        target = property(__target.value, __target.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
        __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 202, 12), )


        body = property(__body.value, __body.set, None, None)


        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_name', pyxb.binding.datatypes.string, required=True)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 204, 8)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 204, 8)

        name = property(__name.value, __name.set, None, None)


        # Attribute type uses Python identifier type
        __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_type', DeviceType, required=True)
        __type._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 205, 8)
        __type._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 205, 8)

        type = property(__type.value, __type.set, None, None)

        _ElementMap.update({
            __neurons.name() : __neurons,
            __connector.name() : __connector,
            __connectorRef.name() : __connectorRef,
            __synapseDynamics.name() : __synapseDynamics,
            __synapseDynamicsRef.name() : __synapseDynamicsRef,
            __target.name() : __target,
            __body.name() : __body
        })
        _AttributeMap.update({
            __name.name() : __name,
            __type.name() : __type
        })
    Namespace.addCategoryObject('typeBinding', 'DeviceChannel', DeviceChannel)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceGroupChannel with content type ELEMENT_ONLY
    class DeviceGroupChannel (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceGroupChannel with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'DeviceGroupChannel')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 208, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}neurons uses Python identifier neurons
        __neurons = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'neurons'), 'neurons', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIneurons', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 210, 12), )


        neurons = property(__neurons.value, __neurons.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connector uses Python identifier connector
        __connector = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connector'), 'connector', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIconnector', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 212, 16), )


        connector = property(__connector.value, __connector.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectorRef uses Python identifier connectorRef
        __connectorRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), 'connectorRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIconnectorRef', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 213, 16), )


        connectorRef = property(__connectorRef.value, __connectorRef.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamics uses Python identifier synapseDynamics
        __synapseDynamics = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), 'synapseDynamics', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamics', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 217, 16), )


        synapseDynamics = property(__synapseDynamics.value, __synapseDynamics.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamicsRef uses Python identifier synapseDynamicsRef
        __synapseDynamicsRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), 'synapseDynamicsRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamicsRef', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 219, 16), )


        synapseDynamicsRef = property(__synapseDynamicsRef.value, __synapseDynamicsRef.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}target uses Python identifier target
        __target = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'target'), 'target', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBItarget', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 222, 12), )


        target = property(__target.value, __target.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
        __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 223, 12), )


        body = property(__body.value, __body.set, None, None)


        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_name', pyxb.binding.datatypes.string, required=True)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 225, 8)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 225, 8)

        name = property(__name.value, __name.set, None, None)


        # Attribute type uses Python identifier type
        __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_type', DeviceType, required=True)
        __type._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 226, 8)
        __type._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 226, 8)

        type = property(__type.value, __type.set, None, None)

        _ElementMap.update({
            __neurons.name() : __neurons,
            __connector.name() : __connector,
            __connectorRef.name() : __connectorRef,
            __synapseDynamics.name() : __synapseDynamics,
            __synapseDynamicsRef.name() : __synapseDynamicsRef,
            __target.name() : __target,
            __body.name() : __body
        })
        _AttributeMap.update({
            __name.name() : __name,
            __type.name() : __type
        })
    Namespace.addCategoryObject('typeBinding', 'DeviceGroupChannel', DeviceGroupChannel)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ChainSelector with content type ELEMENT_ONLY
    class ChainSelector (NeuronGroupSelector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ChainSelector with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ChainSelector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 232, 4)
        _ElementMap = NeuronGroupSelector._ElementMap.copy()
        _AttributeMap = NeuronGroupSelector._AttributeMap.copy()
        # Base type is NeuronGroupSelector

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}neurons uses Python identifier neurons
        __neurons = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'neurons'), 'neurons', '__httpschemas_humanbrainproject_euSP102014BIBI_ChainSelector_httpschemas_humanbrainproject_euSP102014BIBIneurons', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 236, 20), )


        neurons = property(__neurons.value, __neurons.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectors uses Python identifier connectors
        __connectors = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectors'), 'connectors', '__httpschemas_humanbrainproject_euSP102014BIBI_ChainSelector_httpschemas_humanbrainproject_euSP102014BIBIconnectors', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 238, 20), )


        connectors = property(__connectors.value, __connectors.set, None, None)

        _ElementMap.update({
            __neurons.name() : __neurons,
            __connectors.name() : __connectors
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'ChainSelector', ChainSelector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}MapSelector with content type ELEMENT_ONLY
    class MapSelector (NeuronGroupSelector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}MapSelector with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'MapSelector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 245, 4)
        _ElementMap = NeuronGroupSelector._ElementMap.copy()
        _AttributeMap = NeuronGroupSelector._AttributeMap.copy()
        # Base type is NeuronGroupSelector

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}source uses Python identifier source
        __source = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'source'), 'source', '__httpschemas_humanbrainproject_euSP102014BIBI_MapSelector_httpschemas_humanbrainproject_euSP102014BIBIsource', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 249, 20), )


        source = property(__source.value, __source.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}pattern uses Python identifier pattern
        __pattern = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'pattern'), 'pattern', '__httpschemas_humanbrainproject_euSP102014BIBI_MapSelector_httpschemas_humanbrainproject_euSP102014BIBIpattern', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 251, 20), )


        pattern = property(__pattern.value, __pattern.set, None, None)

        _ElementMap.update({
            __source.name() : __source,
            __pattern.name() : __pattern
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'MapSelector', MapSelector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Index with content type EMPTY
    class Index (NeuronSelector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Index with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Index')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 262, 4)
        _ElementMap = NeuronSelector._ElementMap.copy()
        _AttributeMap = NeuronSelector._AttributeMap.copy()
        # Base type is NeuronSelector

        # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector

        # Attribute index uses Python identifier index
        __index = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'index'), 'index', '__httpschemas_humanbrainproject_euSP102014BIBI_Index_index', pyxb.binding.datatypes.nonNegativeInteger, required=True)
        __index._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 265, 16)
        __index._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 265, 16)

        index = property(__index.value, __index.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __index.name() : __index
        })
    Namespace.addCategoryObject('typeBinding', 'Index', Index)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}MultiNeuronSelector with content type EMPTY
    class MultiNeuronSelector (NeuronSelector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}MultiNeuronSelector with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'MultiNeuronSelector')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 270, 4)
        _ElementMap = NeuronSelector._ElementMap.copy()
        _AttributeMap = NeuronSelector._AttributeMap.copy()
        # Base type is NeuronSelector

        # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'MultiNeuronSelector', MultiNeuronSelector)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}IndexTemplate with content type EMPTY
    class IndexTemplate (NeuronSelectorTemplate):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}IndexTemplate with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'IndexTemplate')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 313, 4)
        _ElementMap = NeuronSelectorTemplate._ElementMap.copy()
        _AttributeMap = NeuronSelectorTemplate._AttributeMap.copy()
        # Base type is NeuronSelectorTemplate

        # Attribute index uses Python identifier index
        __index = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'index'), 'index', '__httpschemas_humanbrainproject_euSP102014BIBI_IndexTemplate_index', TemplatePattern, required=True)
        __index._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 316, 16)
        __index._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 316, 16)

        index = property(__index.value, __index.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __index.name() : __index
        })
    Namespace.addCategoryObject('typeBinding', 'IndexTemplate', IndexTemplate)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}RangeTemplate with content type EMPTY
    class RangeTemplate (NeuronSelectorTemplate):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}RangeTemplate with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RangeTemplate')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 321, 4)
        _ElementMap = NeuronSelectorTemplate._ElementMap.copy()
        _AttributeMap = NeuronSelectorTemplate._AttributeMap.copy()
        # Base type is NeuronSelectorTemplate

        # Attribute from uses Python identifier from_
        __from = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'from'), 'from_', '__httpschemas_humanbrainproject_euSP102014BIBI_RangeTemplate_from', TemplatePattern, required=True)
        __from._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 324, 16)
        __from._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 324, 16)

        from_ = property(__from.value, __from.set, None, None)


        # Attribute to uses Python identifier to
        __to = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'to'), 'to', '__httpschemas_humanbrainproject_euSP102014BIBI_RangeTemplate_to', TemplatePattern, required=True)
        __to._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 325, 16)
        __to._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 325, 16)

        to = property(__to.value, __to.set, None, None)


        # Attribute step uses Python identifier step
        __step = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'step'), 'step', '__httpschemas_humanbrainproject_euSP102014BIBI_RangeTemplate_step', TemplatePattern)
        __step._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 326, 16)
        __step._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 326, 16)

        step = property(__step.value, __step.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __from.name() : __from,
            __to.name() : __to,
            __step.name() : __step
        })
    Namespace.addCategoryObject('typeBinding', 'RangeTemplate', RangeTemplate)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ListTemplate with content type ELEMENT_ONLY
    class ListTemplate (NeuronSelectorTemplate):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ListTemplate with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ListTemplate')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 331, 4)
        _ElementMap = NeuronSelectorTemplate._ElementMap.copy()
        _AttributeMap = NeuronSelectorTemplate._AttributeMap.copy()
        # Base type is NeuronSelectorTemplate

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}element uses Python identifier element
        __element = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'element'), 'element', '__httpschemas_humanbrainproject_euSP102014BIBI_ListTemplate_httpschemas_humanbrainproject_euSP102014BIBIelement', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 335, 20), )


        element = property(__element.value, __element.set, None, None)

        _ElementMap.update({
            __element.name() : __element
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'ListTemplate', ListTemplate)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TopicChannel with content type ELEMENT_ONLY
    class TopicChannel (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TopicChannel with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TopicChannel')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 355, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
        __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 357, 12), )


        body = property(__body.value, __body.set, None, None)


        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_name', pyxb.binding.datatypes.string)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 359, 8)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 359, 8)

        name = property(__name.value, __name.set, None, None)


        # Attribute topic uses Python identifier topic
        __topic = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'topic'), 'topic', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_topic', RobotTopicAddress)
        __topic._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 360, 8)
        __topic._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 360, 8)

        topic = property(__topic.value, __topic.set, None, None)


        # Attribute type uses Python identifier type
        __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_type', pyxb.binding.datatypes.string)
        __type._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 361, 8)
        __type._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 361, 8)

        type = property(__type.value, __type.set, None, None)

        _ElementMap.update({
            __body.name() : __body
        })
        _AttributeMap.update({
            __name.name() : __name,
            __topic.name() : __topic,
            __type.name() : __type
        })
    Namespace.addCategoryObject('typeBinding', 'TopicChannel', TopicChannel)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Scale with content type ELEMENT_ONLY
    class Scale (FlowExpression):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Scale with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Scale')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 373, 4)
        _ElementMap = FlowExpression._ElementMap.copy()
        _AttributeMap = FlowExpression._AttributeMap.copy()
        # Base type is FlowExpression

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}inner uses Python identifier inner
        __inner = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'inner'), 'inner', '__httpschemas_humanbrainproject_euSP102014BIBI_Scale_httpschemas_humanbrainproject_euSP102014BIBIinner', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 377, 20), )


        inner = property(__inner.value, __inner.set, None, None)


        # Attribute factor uses Python identifier factor
        __factor = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'factor'), 'factor', '__httpschemas_humanbrainproject_euSP102014BIBI_Scale_factor', pyxb.binding.datatypes.double)
        __factor._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 379, 16)
        __factor._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 379, 16)

        factor = property(__factor.value, __factor.set, None, None)

        _ElementMap.update({
            __inner.name() : __inner
        })
        _AttributeMap.update({
            __factor.name() : __factor
        })
    Namespace.addCategoryObject('typeBinding', 'Scale', Scale)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Call with content type ELEMENT_ONLY
    class Call (FlowExpression):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Call with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Call')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 384, 4)
        _ElementMap = FlowExpression._ElementMap.copy()
        _AttributeMap = FlowExpression._AttributeMap.copy()
        # Base type is FlowExpression

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}argument uses Python identifier argument
        __argument = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'argument'), 'argument', '__httpschemas_humanbrainproject_euSP102014BIBI_Call_httpschemas_humanbrainproject_euSP102014BIBIargument', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 388, 20), )


        argument = property(__argument.value, __argument.set, None, None)


        # Attribute type uses Python identifier type
        __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_Call_type', pyxb.binding.datatypes.string)
        __type._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 390, 16)
        __type._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 390, 16)

        type = property(__type.value, __type.set, None, None)

        _ElementMap.update({
            __argument.name() : __argument
        })
        _AttributeMap.update({
            __type.name() : __type
        })
    Namespace.addCategoryObject('typeBinding', 'Call', Call)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator with content type ELEMENT_ONLY
    class Operator (FlowExpression):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = True
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Operator')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 395, 4)
        _ElementMap = FlowExpression._ElementMap.copy()
        _AttributeMap = FlowExpression._AttributeMap.copy()
        # Base type is FlowExpression

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand uses Python identifier operand
        __operand = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'operand'), 'operand', '__httpschemas_humanbrainproject_euSP102014BIBI_Operator_httpschemas_humanbrainproject_euSP102014BIBIoperand', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20), )


        operand = property(__operand.value, __operand.set, None, None)

        _ElementMap.update({
            __operand.name() : __operand
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Operator', Operator)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SimulationStep with content type EMPTY
    class SimulationStep (FlowExpression):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SimulationStep with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SimulationStep')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 456, 4)
        _ElementMap = FlowExpression._ElementMap.copy()
        _AttributeMap = FlowExpression._AttributeMap.copy()
        # Base type is FlowExpression
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'SimulationStep', SimulationStep)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ArgumentReference with content type EMPTY
    class ArgumentReference (FlowExpression):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ArgumentReference with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ArgumentReference')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 463, 4)
        _ElementMap = FlowExpression._ElementMap.copy()
        _AttributeMap = FlowExpression._AttributeMap.copy()
        # Base type is FlowExpression

        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_ArgumentReference_name', pyxb.binding.datatypes.string, required=True)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 466, 16)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 466, 16)

        name = property(__name.value, __name.set, None, None)


        # Attribute property uses Python identifier property_
        __property = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'property'), 'property_', '__httpschemas_humanbrainproject_euSP102014BIBI_ArgumentReference_property', pyxb.binding.datatypes.string)
        __property._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 467, 16)
        __property._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 467, 16)

        property_ = property(__property.value, __property.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __name.name() : __name,
            __property.name() : __property
        })
    Namespace.addCategoryObject('typeBinding', 'ArgumentReference', ArgumentReference)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Constant with content type EMPTY
    class Constant (FlowExpression):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Constant with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Constant')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 472, 4)
        _ElementMap = FlowExpression._ElementMap.copy()
        _AttributeMap = FlowExpression._AttributeMap.copy()
        # Base type is FlowExpression

        # Attribute value uses Python identifier value_
        __value = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'value'), 'value_', '__httpschemas_humanbrainproject_euSP102014BIBI_Constant_value', pyxb.binding.datatypes.double, required=True)
        __value._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 475, 16)
        __value._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 475, 16)

        value_ = property(__value.value, __value.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __value.name() : __value
        })
    Namespace.addCategoryObject('typeBinding', 'Constant', Constant)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ConstantString with content type EMPTY
    class ConstantString (FlowExpression):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ConstantString with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConstantString')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 480, 4)
        _ElementMap = FlowExpression._ElementMap.copy()
        _AttributeMap = FlowExpression._AttributeMap.copy()
        # Base type is FlowExpression

        # Attribute value uses Python identifier value_
        __value = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'value'), 'value_', '__httpschemas_humanbrainproject_euSP102014BIBI_ConstantString_value', pyxb.binding.datatypes.string, required=True)
        __value._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 483, 16)
        __value._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 483, 16)

        value_ = property(__value.value, __value.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __value.name() : __value
        })
    Namespace.addCategoryObject('typeBinding', 'ConstantString', ConstantString)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Robot2Neuron with content type ELEMENT_ONLY
    class Robot2Neuron (BIBITransferFunction):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Robot2Neuron with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Robot2Neuron')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 89, 4)
        _ElementMap = BIBITransferFunction._ElementMap.copy()
        _AttributeMap = BIBITransferFunction._AttributeMap.copy()
        # Base type is BIBITransferFunction

        # Element local ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element device ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element deviceGroup ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}deviceGroup) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element topic ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}topic) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}returnValue uses Python identifier returnValue
        __returnValue = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'returnValue'), 'returnValue', '__httpschemas_humanbrainproject_euSP102014BIBI_Robot2Neuron_httpschemas_humanbrainproject_euSP102014BIBIreturnValue', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 93, 20), )


        returnValue = property(__returnValue.value, __returnValue.set, None, None)


        # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
        _ElementMap.update({
            __returnValue.name() : __returnValue
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Robot2Neuron', Robot2Neuron)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Neuron2Monitor with content type ELEMENT_ONLY
    class Neuron2Monitor (BIBITransferFunction):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Neuron2Monitor with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Neuron2Monitor')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 100, 4)
        _ElementMap = BIBITransferFunction._ElementMap.copy()
        _AttributeMap = BIBITransferFunction._AttributeMap.copy()
        # Base type is BIBITransferFunction

        # Element local ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element device ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Attribute name is restricted from parent

        # Attribute name uses Python identifier name
        __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_name', pyxb.binding.datatypes.string)
        __name._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 109, 16)
        __name._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 109, 16)

        name = property(__name.value, __name.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __name.name() : __name
        })
    Namespace.addCategoryObject('typeBinding', 'Neuron2Monitor', Neuron2Monitor)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Neuron2Robot with content type ELEMENT_ONLY
    class Neuron2Robot (BIBITransferFunction):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Neuron2Robot with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Neuron2Robot')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 114, 4)
        _ElementMap = BIBITransferFunction._ElementMap.copy()
        _AttributeMap = BIBITransferFunction._AttributeMap.copy()
        # Base type is BIBITransferFunction

        # Element local ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element device ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element deviceGroup ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}deviceGroup) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element topic ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}topic) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}returnValue uses Python identifier returnValue
        __returnValue = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'returnValue'), 'returnValue', '__httpschemas_humanbrainproject_euSP102014BIBI_Neuron2Robot_httpschemas_humanbrainproject_euSP102014BIBIreturnValue', False, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 118, 20), )


        returnValue = property(__returnValue.value, __returnValue.set, None, None)


        # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
        _ElementMap.update({
            __returnValue.name() : __returnValue
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Neuron2Robot', Neuron2Robot)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Range with content type EMPTY
    class Range (MultiNeuronSelector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Range with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Range')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 276, 4)
        _ElementMap = MultiNeuronSelector._ElementMap.copy()
        _AttributeMap = MultiNeuronSelector._AttributeMap.copy()
        # Base type is MultiNeuronSelector

        # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector

        # Attribute from uses Python identifier from_
        __from = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'from'), 'from_', '__httpschemas_humanbrainproject_euSP102014BIBI_Range_from', pyxb.binding.datatypes.nonNegativeInteger, required=True)
        __from._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 279, 16)
        __from._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 279, 16)

        from_ = property(__from.value, __from.set, None, None)


        # Attribute to uses Python identifier to
        __to = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'to'), 'to', '__httpschemas_humanbrainproject_euSP102014BIBI_Range_to', pyxb.binding.datatypes.nonNegativeInteger, required=True)
        __to._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 280, 16)
        __to._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 280, 16)

        to = property(__to.value, __to.set, None, None)


        # Attribute step uses Python identifier step
        __step = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'step'), 'step', '__httpschemas_humanbrainproject_euSP102014BIBI_Range_step', pyxb.binding.datatypes.nonNegativeInteger)
        __step._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 281, 16)
        __step._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 281, 16)

        step = property(__step.value, __step.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __from.name() : __from,
            __to.name() : __to,
            __step.name() : __step
        })
    Namespace.addCategoryObject('typeBinding', 'Range', Range)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}List with content type ELEMENT_ONLY
    class List (MultiNeuronSelector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}List with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'List')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 286, 4)
        _ElementMap = MultiNeuronSelector._ElementMap.copy()
        _AttributeMap = MultiNeuronSelector._AttributeMap.copy()
        # Base type is MultiNeuronSelector

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}element uses Python identifier element
        __element = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'element'), 'element', '__httpschemas_humanbrainproject_euSP102014BIBI_List_httpschemas_humanbrainproject_euSP102014BIBIelement', True, pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 290, 20), )


        element = property(__element.value, __element.set, None, None)


        # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector
        _ElementMap.update({
            __element.name() : __element
        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'List', List)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Population with content type EMPTY
    class Population (MultiNeuronSelector):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Population with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Population')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 296, 4)
        _ElementMap = MultiNeuronSelector._ElementMap.copy()
        _AttributeMap = MultiNeuronSelector._AttributeMap.copy()
        # Base type is MultiNeuronSelector

        # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector

        # Attribute count uses Python identifier count
        __count = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'count'), 'count', '__httpschemas_humanbrainproject_euSP102014BIBI_Population_count', pyxb.binding.datatypes.nonNegativeInteger)
        __count._DeclarationLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 299, 16)
        __count._UseLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 299, 16)

        count = property(__count.value, __count.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __count.name() : __count
        })
    Namespace.addCategoryObject('typeBinding', 'Population', Population)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Add with content type ELEMENT_ONLY
    class Add (Operator):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Add with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Add')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 406, 4)
        _ElementMap = Operator._ElementMap.copy()
        _AttributeMap = Operator._AttributeMap.copy()
        # Base type is Operator

        # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Add', Add)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Subtract with content type ELEMENT_ONLY
    class Subtract (Operator):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Subtract with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Subtract')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 412, 4)
        _ElementMap = Operator._ElementMap.copy()
        _AttributeMap = Operator._AttributeMap.copy()
        # Base type is Operator

        # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Subtract', Subtract)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Multiply with content type ELEMENT_ONLY
    class Multiply (Operator):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Multiply with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Multiply')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 418, 4)
        _ElementMap = Operator._ElementMap.copy()
        _AttributeMap = Operator._AttributeMap.copy()
        # Base type is Operator

        # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Multiply', Multiply)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Divide with content type ELEMENT_ONLY
    class Divide (Operator):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Divide with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Divide')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 424, 4)
        _ElementMap = Operator._ElementMap.copy()
        _AttributeMap = Operator._AttributeMap.copy()
        # Base type is Operator

        # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Divide', Divide)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Min with content type ELEMENT_ONLY
    class Min (Operator):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Min with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Min')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 430, 4)
        _ElementMap = Operator._ElementMap.copy()
        _AttributeMap = Operator._AttributeMap.copy()
        # Base type is Operator

        # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Min', Min)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Max with content type ELEMENT_ONLY
    class Max (Operator):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Max with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Max')
        _XSDLocation = pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 436, 4)
        _ElementMap = Operator._ElementMap.copy()
        _AttributeMap = Operator._AttributeMap.copy()
        # Base type is Operator

        # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
        _ElementMap.update({

        })
        _AttributeMap.update({

        })
    Namespace.addCategoryObject('typeBinding', 'Max', Max)


    bibi = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'bibi'), BIBIConfiguration, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 8, 4))
    Namespace.addCategoryObject('elementBinding', bibi.name().localName(), bibi)



    BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'brainModel'), BrainModel, scope=BIBIConfiguration, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 11, 12)))

    BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'bodyModel'), SDF_Filename, scope=BIBIConfiguration, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 12, 12)))

    BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'extRobotController'), Script_Filename, scope=BIBIConfiguration, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 13, 12)))

    BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectors'), NeuronConnector, scope=BIBIConfiguration, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 15, 12)))

    BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), SynapseDynamics, scope=BIBIConfiguration, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 17, 12)))

    BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'transferFunction'), TransferFunction, scope=BIBIConfiguration, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 19, 12)))

    def _BuildAutomaton ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton
        del _BuildAutomaton
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 13, 12))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 15, 12))
        counters.add(cc_1)
        cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 17, 12))
        counters.add(cc_2)
        cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 19, 12))
        counters.add(cc_3)
        states = []
        final_update = None
        symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'brainModel')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 11, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'bodyModel')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 12, 12))
        st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'extRobotController')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 13, 12))
        st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_2)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectors')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 15, 12))
        st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_3)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_2, False))
        symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 17, 12))
        st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_4)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_3, False))
        symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'transferFunction')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 19, 12))
        st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_5)
        transitions = []
        transitions.append(fac.Transition(st_1, [
             ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_2, [
             ]))
        transitions.append(fac.Transition(st_3, [
             ]))
        transitions.append(fac.Transition(st_4, [
             ]))
        transitions.append(fac.Transition(st_5, [
             ]))
        st_1._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_2._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_3._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_2, True) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_2, False) ]))
        st_4._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_3, True) ]))
        st_5._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    BIBIConfiguration._Automaton = _BuildAutomaton()




    BrainModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'file'), Brain_Filename, scope=BrainModel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 26, 12)))

    BrainModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'populations'), MultiNeuronSelector, scope=BrainModel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 27, 12)))

    def _BuildAutomaton_ ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_
        del _BuildAutomaton_
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 27, 12))
        counters.add(cc_0)
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(BrainModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'file')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 26, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(BrainModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'populations')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 27, 12))
        st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        transitions = []
        transitions.append(fac.Transition(st_1, [
             ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_1._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    BrainModel._Automaton = _BuildAutomaton_()




    Argument._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'value'), FlowExpression, scope=Argument, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 444, 12)))

    def _BuildAutomaton_2 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_2
        del _BuildAutomaton_2
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(Argument._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'value')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 444, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Argument._Automaton = _BuildAutomaton_2()




    Local._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=Local, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 451, 12)))

    def _BuildAutomaton_3 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_3
        del _BuildAutomaton_3
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(Local._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 451, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Local._Automaton = _BuildAutomaton_3()




    def _BuildAutomaton_4 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_4
        del _BuildAutomaton_4
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        return fac.Automaton(states, counters, True, containing_state=None)
    PythonTransferFunction._Automaton = _BuildAutomaton_4()




    BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'local'), Local, scope=BIBITransferFunction, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20)))

    BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'device'), DeviceChannel, scope=BIBITransferFunction, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 78, 24)))

    BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup'), DeviceGroupChannel, scope=BIBITransferFunction, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 79, 24)))

    BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'topic'), TopicChannel, scope=BIBITransferFunction, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20)))

    def _BuildAutomaton_5 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_5
        del _BuildAutomaton_5
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 77, 20))
        counters.add(cc_1)
        cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20))
        counters.add(cc_2)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 78, 24))
        st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 79, 24))
        st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_2)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_2, False))
        symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'topic')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20))
        st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_3)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_1._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_2._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_2, True) ]))
        st_3._set_transitionSet(transitions)
        return fac.Automaton(states, counters, True, containing_state=None)
    BIBITransferFunction._Automaton = _BuildAutomaton_5()




    DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'neurons'), NeuronSelector, scope=DeviceChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 189, 12)))

    DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connector'), NeuronConnector, scope=DeviceChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 191, 16)))

    DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), NeuronConnectorRef, scope=DeviceChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 192, 16)))

    DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), SynapseDynamics, scope=DeviceChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 196, 16)))

    DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), SynapseDynamicsRef, scope=DeviceChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 198, 16)))

    DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'target'), NeuronTarget, scope=DeviceChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 201, 12)))

    DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=DeviceChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 202, 12)))

    def _BuildAutomaton_6 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_6
        del _BuildAutomaton_6
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 190, 12))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 195, 12))
        counters.add(cc_1)
        cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 201, 12))
        counters.add(cc_2)
        cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 202, 12))
        counters.add(cc_3)
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'neurons')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 189, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connector')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 191, 16))
        st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectorRef')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 192, 16))
        st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_2)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 196, 16))
        st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_3)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 198, 16))
        st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_4)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_2, False))
        symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'target')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 201, 12))
        st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_5)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_3, False))
        symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 202, 12))
        st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_6)
        transitions = []
        transitions.append(fac.Transition(st_1, [
             ]))
        transitions.append(fac.Transition(st_2, [
             ]))
        transitions.append(fac.Transition(st_3, [
             ]))
        transitions.append(fac.Transition(st_4, [
             ]))
        transitions.append(fac.Transition(st_5, [
             ]))
        transitions.append(fac.Transition(st_6, [
             ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_1._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_2._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_3._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_4._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_2, True) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_2, False) ]))
        st_5._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_3, True) ]))
        st_6._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    DeviceChannel._Automaton = _BuildAutomaton_6()




    DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'neurons'), NeuronGroupSelector, scope=DeviceGroupChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 210, 12)))

    DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connector'), NeuronConnector, scope=DeviceGroupChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 212, 16)))

    DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), NeuronConnectorRef, scope=DeviceGroupChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 213, 16)))

    DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), SynapseDynamics, scope=DeviceGroupChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 217, 16)))

    DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), SynapseDynamicsRef, scope=DeviceGroupChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 219, 16)))

    DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'target'), NeuronTarget, scope=DeviceGroupChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 222, 12)))

    DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=DeviceGroupChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 223, 12)))

    def _BuildAutomaton_7 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_7
        del _BuildAutomaton_7
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 211, 12))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 216, 12))
        counters.add(cc_1)
        cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 222, 12))
        counters.add(cc_2)
        cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 223, 12))
        counters.add(cc_3)
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'neurons')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 210, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connector')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 212, 16))
        st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectorRef')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 213, 16))
        st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_2)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 217, 16))
        st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_3)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 219, 16))
        st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_4)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_2, False))
        symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'target')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 222, 12))
        st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_5)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_3, False))
        symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 223, 12))
        st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_6)
        transitions = []
        transitions.append(fac.Transition(st_1, [
             ]))
        transitions.append(fac.Transition(st_2, [
             ]))
        transitions.append(fac.Transition(st_3, [
             ]))
        transitions.append(fac.Transition(st_4, [
             ]))
        transitions.append(fac.Transition(st_5, [
             ]))
        transitions.append(fac.Transition(st_6, [
             ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_1._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_2._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_3._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_4._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_5, [
            fac.UpdateInstruction(cc_2, True) ]))
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_2, False) ]))
        st_5._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_6, [
            fac.UpdateInstruction(cc_3, True) ]))
        st_6._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    DeviceGroupChannel._Automaton = _BuildAutomaton_7()




    ChainSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'neurons'), NeuronSelector, scope=ChainSelector, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 236, 20)))

    ChainSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectors'), NeuronGroupSelector, scope=ChainSelector, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 238, 20)))

    def _BuildAutomaton_8 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_8
        del _BuildAutomaton_8
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 236, 20))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 238, 20))
        counters.add(cc_1)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(ChainSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'neurons')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 236, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(ChainSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectors')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 238, 20))
        st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        st_1._set_transitionSet(transitions)
        return fac.Automaton(states, counters, True, containing_state=None)
    ChainSelector._Automaton = _BuildAutomaton_8()




    MapSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'source'), MultiNeuronSelector, scope=MapSelector, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 249, 20)))

    MapSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'pattern'), NeuronSelectorTemplate, scope=MapSelector, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 251, 20)))

    def _BuildAutomaton_9 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_9
        del _BuildAutomaton_9
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = None
        symbol = pyxb.binding.content.ElementUse(MapSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'source')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 249, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(MapSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'pattern')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 251, 20))
        st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        transitions = []
        transitions.append(fac.Transition(st_1, [
             ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        st_1._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    MapSelector._Automaton = _BuildAutomaton_9()




    ListTemplate._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'element'), TemplatePattern, scope=ListTemplate, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 335, 20)))

    def _BuildAutomaton_10 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_10
        del _BuildAutomaton_10
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(ListTemplate._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'element')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 335, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
             ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    ListTemplate._Automaton = _BuildAutomaton_10()




    TopicChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=TopicChannel, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 357, 12)))

    def _BuildAutomaton_11 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_11
        del _BuildAutomaton_11
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 357, 12))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(TopicChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 357, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, True, containing_state=None)
    TopicChannel._Automaton = _BuildAutomaton_11()




    Scale._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'inner'), FlowExpression, scope=Scale, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 377, 20)))

    def _BuildAutomaton_12 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_12
        del _BuildAutomaton_12
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(Scale._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'inner')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 377, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Scale._Automaton = _BuildAutomaton_12()




    Call._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'argument'), Argument, scope=Call, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 388, 20)))

    def _BuildAutomaton_13 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_13
        del _BuildAutomaton_13
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(Call._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'argument')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 388, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
             ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Call._Automaton = _BuildAutomaton_13()




    Operator._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'operand'), FlowExpression, scope=Operator, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20)))

    def _BuildAutomaton_14 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_14
        del _BuildAutomaton_14
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Operator._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Operator._Automaton = _BuildAutomaton_14()




    Robot2Neuron._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'returnValue'), DeviceChannel, scope=Robot2Neuron, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 93, 20)))

    def _BuildAutomaton_15 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_15
        del _BuildAutomaton_15
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 77, 20))
        counters.add(cc_1)
        cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20))
        counters.add(cc_2)
        cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 93, 20))
        counters.add(cc_3)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 78, 24))
        st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 79, 24))
        st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_2)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_2, False))
        symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'topic')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20))
        st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_3)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_3, False))
        symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'returnValue')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 93, 20))
        st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_4)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_1._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_2._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_2, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_2, False) ]))
        st_3._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_3, True) ]))
        st_4._set_transitionSet(transitions)
        return fac.Automaton(states, counters, True, containing_state=None)
    Robot2Neuron._Automaton = _BuildAutomaton_15()




    def _BuildAutomaton_16 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_16
        del _BuildAutomaton_16
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 104, 20))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 105, 20))
        counters.add(cc_1)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Neuron2Monitor._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 104, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(Neuron2Monitor._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 106, 24))
        st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        st_1._set_transitionSet(transitions)
        return fac.Automaton(states, counters, True, containing_state=None)
    Neuron2Monitor._Automaton = _BuildAutomaton_16()




    Neuron2Robot._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'returnValue'), TopicChannel, scope=Neuron2Robot, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 118, 20)))

    def _BuildAutomaton_17 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_17
        del _BuildAutomaton_17
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 77, 20))
        counters.add(cc_1)
        cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20))
        counters.add(cc_2)
        cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 118, 20))
        counters.add(cc_3)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 76, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 78, 24))
        st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_1, False))
        symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 79, 24))
        st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_2)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_2, False))
        symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'topic')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 81, 20))
        st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_3)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_3, False))
        symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'returnValue')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 118, 20))
        st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_4)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_1._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_1, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_2._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_2, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_2, False) ]))
        st_3._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_3, True) ]))
        st_4._set_transitionSet(transitions)
        return fac.Automaton(states, counters, True, containing_state=None)
    Neuron2Robot._Automaton = _BuildAutomaton_17()




    List._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'element'), pyxb.binding.datatypes.nonNegativeInteger, scope=List, location=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 290, 20)))

    def _BuildAutomaton_18 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_18
        del _BuildAutomaton_18
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(List._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'element')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 290, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
             ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    List._Automaton = _BuildAutomaton_18()




    def _BuildAutomaton_19 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_19
        del _BuildAutomaton_19
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Add._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Add._Automaton = _BuildAutomaton_19()




    def _BuildAutomaton_20 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_20
        del _BuildAutomaton_20
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Subtract._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Subtract._Automaton = _BuildAutomaton_20()




    def _BuildAutomaton_21 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_21
        del _BuildAutomaton_21
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Multiply._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Multiply._Automaton = _BuildAutomaton_21()




    def _BuildAutomaton_22 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_22
        del _BuildAutomaton_22
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Divide._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Divide._Automaton = _BuildAutomaton_22()




    def _BuildAutomaton_23 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_23
        del _BuildAutomaton_23
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Min._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Min._Automaton = _BuildAutomaton_23()




    def _BuildAutomaton_24 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_24
        del _BuildAutomaton_24
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(Max._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/sebastiankrach/bbpcode/Models/BIBI/bibi_configuration.xsd', 399, 20))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    Max._Automaton = _BuildAutomaton_24()

