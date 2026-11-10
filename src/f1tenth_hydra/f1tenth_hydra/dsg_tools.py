"""Small helpers over spark_dsg graphs, shared by the viewer, hydra_cost, the logger and the
smoke test. spark_dsg is imported lazily: this module imports without it."""

# (layer id, partition) -> name, as Hydra builds its graph (Spark-DSG DsgLayers; the RViz
# namespaces layer_3p1_* are the MESH_PLACES partition).
_KNOWN = {(2, 0): 'OBJECTS', (3, 0): 'PLACES', (3, 1): 'MESH_PLACES', (4, 0): 'ROOMS',
          (5, 0): 'BUILDINGS', (1, 0): 'SEGMENTS'}


def import_spark_dsg():
    """spark_dsg, or None with the reason (the bindings live in ~/hydra_ws, docs/hydra_hil.md)."""
    try:
        import spark_dsg
        return spark_dsg, None
    except ImportError as exc:
        return None, (f'spark_dsg not importable ({exc}): source scripts/hydra/hydra_env.sh '
                      '(PYTHONPATH ~/hydra_ws/install/spark_dsg_python)')


def layer_label(layer_view):
    key = getattr(layer_view, 'key', None)
    lid = int(getattr(key, 'layer', getattr(layer_view, 'id', -1)))
    part = int(getattr(key, 'partition', getattr(layer_view, 'partition', 0)) or 0)
    if (lid, part) in _KNOWN:
        return _KNOWN[(lid, part)]
    if lid == 2:
        return 'AGENTS'
    return f'L{lid}' + (f'p{part}' if part else '')


def _layers(G):
    out = list(G.layers)
    try:
        out += list(G.layer_partitions)
    except Exception:  # noqa: BLE001 - older bindings without partitions
        pass
    return out


def graph_summary(G):
    """{'layers': {name: {'nodes': n, 'edges': m}}, 'interlayer_edges': k, 'nodes': N,
    'mesh_vertices': v}"""
    layers = {}
    for layer in _layers(G):
        name = layer_label(layer)
        entry = layers.setdefault(name, {'nodes': 0, 'edges': 0})
        entry['nodes'] += int(layer.num_nodes())
        entry['edges'] += int(layer.num_edges())
    try:
        inter = sum(1 for _ in G.interlayer_edges)
    except Exception:  # noqa: BLE001
        inter = 0
    mesh_vertices = 0
    try:
        if G.has_mesh():
            mesh_vertices = int(G.mesh.num_vertices())
    except Exception:  # noqa: BLE001
        pass
    return {'layers': layers, 'interlayer_edges': inter,
            'nodes': int(G.num_nodes()), 'mesh_vertices': mesh_vertices}


def graph_geometry(G, label_names=None):
    """Nodes and edges as plain data for drawing.

    nodes: [{'key': int, 'id': 'O(12)', 'layer': 'OBJECTS', 'pos': (x, y, z),
             'label': int or None, 'text': str}]
    edges: [(source key, target key, 'intra' | 'inter')], both ends present in nodes
    """
    label_names = label_names or {}
    nodes, known = [], set()
    for layer in _layers(G):
        lname = layer_label(layer)
        for n in layer.nodes:
            p = tuple(float(v) for v in n.attributes.position)
            key = int(n.id.value) if hasattr(n.id, 'value') else int(n.id)
            known.add(key)
            label = getattr(n.attributes, 'semantic_label', None)
            label = None if label is None or label >= 2 ** 31 else int(label)
            name = str(getattr(n.attributes, 'name', '') or '')
            text = name or (label_names.get(label, str(label)) if label is not None else '')
            nodes.append({'key': key, 'id': str(n.id), 'layer': lname, 'pos': p,
                          'label': label, 'text': text})
    edges = []
    for layer in _layers(G):
        for e in layer.edges:
            if int(e.source) in known and int(e.target) in known:
                edges.append((int(e.source), int(e.target), 'intra'))
    try:
        for e in G.interlayer_edges:
            if int(e.source) in known and int(e.target) in known:
                edges.append((int(e.source), int(e.target), 'inter'))
    except Exception:  # noqa: BLE001
        pass
    return nodes, edges
