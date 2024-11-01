import heapq

def dijkstra(graph, start, goal):
    # Inisialisasi
    queue = [(0, start, [])]  # (biaya total, node saat ini, jalur)
    visited = set()  # Menyimpan node yang sudah dikunjungi

    while queue:
        (cost, node, path) = heapq.heappop(queue)  # Ambil node dengan biaya terendah
        if node in visited:
            continue
        
        path = path + [node]
        visited.add(node)

        # Jika mencapai tujuan, cetak jalur dan biaya
        if node == goal:
            print("Jalur Terpendek:", path)
            print("Biaya Total:", cost)
            return path, cost

        # Eksplorasi tetangga
        for (neighbor, weight) in graph[node]:
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path))
    
    print("Tidak ada jalur yang ditemukan")
    return None, None

# Contoh penggunaan
graph = {
    'A': [('B', 1), ('C', 4)],
    'B': [('C', 2), ('D', 5)],
    'C': [('D', 1)],
    'D': []
}
dijkstra(graph, 'A', 'D')