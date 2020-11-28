# Graph hands-on

### BFS
```
import java.util.*;

public class Graph{
  public static ArrayList<Integer> BFSconnected(ArrayList<ArrayList<Integer>> adj, int v, int s){
    boolean visited[] = new boolean[v];
    Queue<Integer> q = new LinkedList<>();
    ArrayList<Integer> al = new ArrayList<>();
    visited[s] = true;
    q.add(s);
    while(!q.isEmpty()){
      int u = q.poll();
      al.add(u);
      for(int n: adj.get(u)){
        if(visited[n]==false){
          q.add(n);
          visited[n] = true;
        }
      }
    }
    return al;
  }
}
```
