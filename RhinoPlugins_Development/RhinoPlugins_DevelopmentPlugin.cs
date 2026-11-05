using Rhino;
using Rhino.PlugIns;

namespace RhinoPlugins_Development
{
    ///<summary>
    /// <para>Every RhinoCommon .rhp assembly must have one and only one PlugIn-derived
    /// class. DO NOT create instances of this class yourself. It is the
    /// responsibility of Rhino to create an instance of this class.</para>
    /// <para>To complete plug-in information, please also see all PlugInDescription
    /// attributes in AssemblyInfo.cs (you might need to click "Project" ->
    /// "Show All Files" to see it in the "Solution Explorer" window).</para>
    ///</summary>
    public class RhinoPlugins_DevelopmentPlugin : PlugIn
    {
        public RhinoPlugins_DevelopmentPlugin()
        {
            Instance = this;
        }

        ///<summary>Gets the only instance of the RhinoPlugins_DevelopmentPlugin plug-in.</summary>
        public static RhinoPlugins_DevelopmentPlugin Instance { get; private set; }

        /// <summary>插件加载时挂接文档事件，并尝试恢复当前文档中的 splat。</summary>
        protected override LoadReturnCode OnLoad(ref string errorMessage)
        {
            RhinoDoc.EndOpenDocument += OnEndOpenDocument;
            RhinoDoc.ActiveDocumentChanged += OnActiveDocumentChanged;

            if (RhinoDoc.ActiveDoc != null)
            {
                try
                {
                    SplatDisplayConduit.ScanAndRestoreSplats(RhinoDoc.ActiveDoc);
                }
                catch
                {
                }
            }

            return LoadReturnCode.Success;
        }

        protected override void OnShutdown()
        {
            RhinoDoc.EndOpenDocument -= OnEndOpenDocument;
            RhinoDoc.ActiveDocumentChanged -= OnActiveDocumentChanged;
            base.OnShutdown();
        }

        private void OnEndOpenDocument(object sender, DocumentOpenEventArgs e)
        {
            if (e.Document != null)
            {
                try
                {
                    SplatDisplayConduit.ScanAndRestoreSplats(e.Document);
                }
                catch
                {
                }
            }
        }

        private void OnActiveDocumentChanged(object sender, DocumentEventArgs e)
        {
            if (e.Document != null)
            {
                try
                {
                    SplatDisplayConduit.ScanAndRestoreSplats(e.Document);
                }
                catch
                {
                }
            }
        }
    }
}
