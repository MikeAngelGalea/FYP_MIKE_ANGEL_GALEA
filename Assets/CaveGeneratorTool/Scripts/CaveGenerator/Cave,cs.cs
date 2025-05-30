// Cave.cs
using System.Collections.Generic;
using UnityEngine;

// If CaveTile lives in some namespace, add that `using` here!
public class Cave : MonoBehaviour
{
    [Tooltip("Scale that was used to generate this cave")]
    public float Scale;

    [Tooltip("Flattened list of cave tiles (returned by MatrixToTileList)")]
    public List<CaveTile> Data;    // <-- change from List<int> to List<CaveTile>

    [Tooltip("Number of columns in the generated map")]
    public int Width;

    [Tooltip("Number of rows in the generated map")]
    public int Height;
}
