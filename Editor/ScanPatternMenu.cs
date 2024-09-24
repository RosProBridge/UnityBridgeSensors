using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.Networking;
using UnitySensors.Sensor.LiDAR;
using Unity.EditorCoroutines.Editor;

public class ScanPatternMenu : EditorWindow
{
    private int currentTab = 0;
    private readonly string[] tabTitles = { "CSV", "Manual", "Prebuilt" };

    private TextAsset _csvFile;
    [SerializeField] private TextAsset _patternsRepo;

    private enum Direction
    {
        CW,
        CCW
    }

    [SerializeField] private Direction _direction;
    [SerializeField] private float[] _zenithAngles;
    [SerializeField] private float _minAzimuthAngle;
    [SerializeField] private float _maxAzimuthAngle;
    [SerializeField] private int _azimuthAngleResolution = 360;
    [SerializeField] private float _zenithAngleOffset = 0;

    private Vector2 _manualMenuScrollPosition = Vector2.zero;
    private Vector2 _prebuiltMenuScrollPosition = Vector2.zero;
    private SerializedObject _so;
    private string localFolderPath;

    private Dictionary<string, string> _patternsRepoDict;
    private Dictionary<string, bool> _isPatternDownloading;

    [MenuItem("ProBridge/Sensors/Add ScanPattern")]
    public static void ShowWindow()             
    {
        GetWindow(typeof(ScanPatternMenu));
    }
    
    private void OnEnable()
    {
        _so = new SerializedObject(this);
        _patternsRepoDict = new Dictionary<string, string>();
        _isPatternDownloading = new Dictionary<string, bool>();
        localFolderPath = Path.Combine(Application.dataPath, "ScanPatterns");
        ReadPatternsRepo();
    }
    

    private void OnGUI()
    {
        currentTab = GUILayout.Toolbar(currentTab, tabTitles, GUILayout.Height(25));

        GUILayout.Space(10);
        switch (currentTab)
        {
            case 0:
                DrawCSVTab();
                break;
            case 1:
                DrawManualTab();
                break;
            case 2:
                DrawPrebuiltTab();
                break;
        }
    }

    private void DrawCSVTab()
    {
        _csvFile = EditorGUILayout.ObjectField("CSV File", _csvFile, typeof(TextAsset), true) as TextAsset;
        _zenithAngleOffset = EditorGUILayout.FloatField("Zenith Angle Offset", _zenithAngleOffset);

        GUILayout.FlexibleSpace();

        if (GUILayout.Button("Generate"))
        {
            GenerateFromCSV();
        }
    }

    private void DrawManualTab()
    {
        _manualMenuScrollPosition = EditorGUILayout.BeginScrollView(_manualMenuScrollPosition);
        _so.Update();
        EditorGUILayout.PropertyField(_so.FindProperty("_zenithAngles"), true);
        _so.ApplyModifiedProperties();
        EditorGUILayout.EndScrollView();

        _minAzimuthAngle = EditorGUILayout.FloatField("Min Azimuth Angle", _minAzimuthAngle);
        _maxAzimuthAngle = EditorGUILayout.FloatField("Max Azimuth Angle", _maxAzimuthAngle);
        _azimuthAngleResolution = EditorGUILayout.IntField("Azimuth Angle Resolution", _azimuthAngleResolution);

        if (GUILayout.Button("Generate"))
        {
            GenerateFromSpecification();
        }
    }

    private void DrawPrebuiltTab()
    {
        GUILayout.Label("Available Patterns: ", EditorStyles.boldLabel);
        EditorGUILayout.Space();
        GUIStyle patternsStyle = new GUIStyle(GUI.skin.box);
        patternsStyle.padding = new RectOffset(10, 10, 10, 10); 
        patternsStyle.margin = new RectOffset(5, 5, 5, 5);
        GUILayout.BeginVertical(patternsStyle);
        _prebuiltMenuScrollPosition = EditorGUILayout.BeginScrollView(_prebuiltMenuScrollPosition);
        foreach (var pattern in _patternsRepoDict)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(pattern.Key);
            GUILayout.FlexibleSpace();
            if (IsPatternDownloaded(pattern.Key))
            {
                GUILayout.Label("Downloaded");
            }
            else if (_isPatternDownloading[pattern.Key])
            {
                GUILayout.Label("Downlaoding...");
            }
            else
            {
                if (GUILayout.Button("Download"))
                {
                    EditorCoroutineUtility.StartCoroutine(DownloadPattern(pattern.Key), this);
                }
            }
            GUILayout.EndHorizontal();
            EditorGUILayout.Space();
            GUILayout.Box("", GUILayout.ExpandWidth(true), GUILayout.Height(1));
            EditorGUILayout.Space();

        }
        
        EditorGUILayout.EndScrollView();
        GUILayout.EndVertical();
    }

    private bool IsPatternDownloaded(string patternName)
    {
        var filePath = Path.Combine(localFolderPath, $"{patternName}.asset");
        return File.Exists(filePath);
    }


    private void GenerateFromCSV()
    {
        if (!_csvFile)
        {
            Debug.LogWarning(this.name + ": CSV file is not set.");
            return;
        }

        string fileText = _csvFile.text;
        string[] lines = fileText.Split('\n');
        string[] headers = lines[0].Split(',');

        int azimuth_index = -1;
        int zenith_index = -1;

        for (int c = 0; c < headers.Length; c++)
        {
            string header = headers[c].ToLower();
            if (header.Contains("zenith")) zenith_index = c;
            else if (header.Contains("azimuth")) azimuth_index = c;
        }

        if (azimuth_index == -1 || zenith_index == -1)
        {
            Debug.LogWarning(this.name + ": Cannot find \"azimuth\" or \"zenith\" header.");
            return;
        }

        ScanPattern scan = ScriptableObject.CreateInstance<ScanPattern>();
        scan.size = lines.Length - 2;
        scan.scans = new float3[scan.size];
        scan.minAzimuthAngle = float.MaxValue;
        scan.maxAzimuthAngle = float.MinValue;
        scan.minZenithAngle = float.MaxValue;
        scan.maxZenithAngle = float.MinValue;

        for (int l = 1; l < lines.Length - 1; l++)
        {
            string[] line = lines[l].Split(',');

            if (line.Length != headers.Length)
            {
                Debug.LogWarning(this.name + "Number of columns does not match.");
                return;
            }

            float azimuthAngle = float.Parse(line[azimuth_index]);
            float zenithAngle = float.Parse(line[zenith_index]) - _zenithAngleOffset;

            scan.minAzimuthAngle = Mathf.Min(scan.minAzimuthAngle, azimuthAngle);
            scan.maxAzimuthAngle = Mathf.Max(scan.maxAzimuthAngle, azimuthAngle);
            scan.minZenithAngle = Mathf.Min(scan.minZenithAngle, zenithAngle);
            scan.maxZenithAngle = Mathf.Max(scan.maxZenithAngle, zenithAngle);

            scan.scans[l - 1] = Quaternion.Euler(zenithAngle, azimuthAngle, 0) * Vector3.forward;
        }

        string localFolderPath = Path.Combine(Application.dataPath, "ScanPatterns");

        if (!Directory.Exists(localFolderPath))
        {
            Directory.CreateDirectory(localFolderPath);
        }

        
        string fileName = "NewScanPattern";
        string filePath = "Assets/ScanPatterns/" + fileName + ".asset";

        int i = 1;
        while (File.Exists(filePath))
        {
            filePath = "Assets/ScanPatterns/" + fileName + i + ".asset";
            i++;
        }

        AssetDatabase.CreateAsset(scan, filePath);
        AssetDatabase.SaveAssets();

        Debug.Log($"New Scan pattern created at: {filePath}");
    }

    private void GenerateFromSpecification()
    {
        if (_zenithAngles == null || _zenithAngles.Length == 0 || _azimuthAngleResolution <= 0) return;

        ScanPattern scan = ScriptableObject.CreateInstance<ScanPattern>();

        scan.size = _zenithAngles.Length * _azimuthAngleResolution;
        scan.scans = new float3[scan.size];

        int index = 0;
        for (int azimuth = 0; azimuth < _azimuthAngleResolution; azimuth++)
        {
            float azimuthAngle = Mathf.Lerp(_minAzimuthAngle, _maxAzimuthAngle,
                (float)(_direction == Direction.CW ? azimuth : _azimuthAngleResolution - 1 - azimuth) /
                _azimuthAngleResolution);
            foreach (float zenithAngle in _zenithAngles)
            {
                scan.scans[index] = Quaternion.Euler(-zenithAngle, azimuthAngle, 0) * Vector3.forward;
                index++;
            }
        }

        scan.minAzimuthAngle = _minAzimuthAngle;
        scan.maxAzimuthAngle = _maxAzimuthAngle;

        scan.minZenithAngle = float.MaxValue;
        scan.maxZenithAngle = float.MinValue;
        foreach (float zenithAngle in _zenithAngles)
        {
            scan.minZenithAngle = Mathf.Min(scan.minZenithAngle, zenithAngle);
            scan.maxZenithAngle = Mathf.Max(scan.maxZenithAngle, zenithAngle);
        }
        
        string localFolderPath = Path.Combine(Application.dataPath, "ScanPatterns");

        if (!Directory.Exists(localFolderPath))
        {
            Directory.CreateDirectory(localFolderPath);
        }

        
        string fileName = "NewScanPattern";
        string filePath = "Assets/ScanPatterns/" + fileName + ".asset";

        int i = 1;
        while (File.Exists(filePath))
        {
            filePath = "Assets/ScanPatterns/" + fileName + i + ".asset";
            i++;
        }

        AssetDatabase.CreateAsset(scan, filePath);
        AssetDatabase.SaveAssets();

        Debug.Log($"New Scan pattern created at: {filePath}");
    }

    IEnumerator DownloadPattern(string patternName)
    {
        string url = _patternsRepoDict[patternName];
        Debug.Log($"Downloading {patternName} from {url}...");

        _isPatternDownloading[patternName] = true;
        using (UnityWebRequest webRequest = UnityWebRequest.Get(url))
        {
            yield return webRequest.SendWebRequest();
            
            if (webRequest.result == UnityWebRequest.Result.ConnectionError || webRequest.result == UnityWebRequest.Result.ProtocolError)
            {
                Debug.LogError($"Error downloading file: {webRequest.error}");
                _isPatternDownloading[patternName] = false;
            }
            else
            {
                var filePath = Path.Combine(localFolderPath, $"{patternName}.asset");
                File.WriteAllBytes(filePath, webRequest.downloadHandler.data);
                Debug.Log($"File downloaded successfully to {filePath}");
                AssetDatabase.Refresh();
                _isPatternDownloading[patternName] = false;
            }
        }
    }

    private void ReadPatternsRepo()
    {
        var lines = _patternsRepo.text.Split('\n');

        foreach (var line in lines)
        {
            var fields = line.Split(',');
            _patternsRepoDict.Add(fields[0], fields[1]);
            _isPatternDownloading.Add(fields[0], false);
        }
    }
    
}