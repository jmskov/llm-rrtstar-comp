use rand::Rng;
use std::time::{Duration, Instant};
use std::collections::BinaryHeap;
use std::cmp::Ordering;
use std::fs::File;
use std::io::Write;
use std::f64::INFINITY;

#[derive(Debug, Clone, Copy)]
struct Point {
    x: f64,
    y: f64,
}

impl Point {
    fn dist(&self, other: &Point) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

#[derive(Debug)]
struct Node {
    point: Point,
    parent: Option<usize>,
    cost: f64,
}

#[derive(Debug, Clone, Copy)]

struct CircleObstacle {
    center: Point,
    radius: f64,
}

#[derive(Debug)]
struct Path {
    nodes: Vec<Point>,
}

struct RRTStar {
    nodes: Vec<Node>,
    start: Point,
    goal: Point,
    obstacle: CircleObstacle,
    max_iter: usize,
    step_size: f64,
    search_radius: f64,
    goal_sample_rate: f64,
    workspace_bounds: (Point, Point),
}

impl RRTStar {
    fn new(
        start: Point,
        goal: Point,
        obstacle: CircleObstacle,
        max_iter: usize,
        step_size: f64,
        search_radius: f64,
        goal_sample_rate: f64,
        workspace_bounds: (Point, Point),
    ) -> Self {
        let mut nodes = Vec::with_capacity(max_iter);
        nodes.push(Node {
            point: start,
            parent: None,
            cost: 0.0,
        });
        
        RRTStar {
            nodes,
            start,
            goal,
            obstacle,
            max_iter,
            step_size,
            search_radius,
            goal_sample_rate,
            workspace_bounds,
        }
    }

    fn random_point(&self) -> Point {
        let mut rng = rand::thread_rng();
        if rng.gen::<f64>() < self.goal_sample_rate {
            self.goal
        } else {
            Point {
                x: rng.gen_range(self.workspace_bounds.0.x..self.workspace_bounds.1.x),
                y: rng.gen_range(self.workspace_bounds.0.y..self.workspace_bounds.1.y),
            }
        }
    }

    fn nearest_neighbor(&self, point: &Point) -> usize {
        let mut nearest = 0;
        let mut min_dist = INFINITY;
        
        for (i, node) in self.nodes.iter().enumerate() {
            let dist = node.point.dist(point);
            if dist < min_dist {
                min_dist = dist;
                nearest = i;
            }
        }
        nearest
    }

    fn steer(&self, from: &Point, to: &Point) -> Point {
        let dist = from.dist(to);
        if dist < self.step_size {
            *to
        } else {
            let theta = (to.y - from.y).atan2(to.x - from.x);
            Point {
                x: from.x + self.step_size * theta.cos(),
                y: from.y + self.step_size * theta.sin(),
            }
        }
    }

    fn collision_free(&self, from: &Point, to: &Point) -> bool {
        let dist = from.dist(to);
        let steps = (dist / (self.step_size * 0.5)).ceil() as usize;
        
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let point = Point {
                x: from.x + (to.x - from.x) * t,
                y: from.y + (to.y - from.y) * t,
            };
            
            if point.dist(&self.obstacle.center) <= self.obstacle.radius {
                return false;
            }
        }
        true
    }
    
    fn calculate_rewire_radius(&self) -> f64 {
        let n = self.nodes.len() as f64;
        let base_radius = self.search_radius; // This is our r_ parameter
        
        // OMPL formula: r * log(n + 1) / (n + 1)
        base_radius * (1.0 + n).ln() / (1.0 + n)
    }

    // Then modify find_neighbors to use this dynamic radius
    fn find_neighbors(&self, point: &Point) -> Vec<usize> {
        let mut neighbors = Vec::new();
        let radius = self.calculate_rewire_radius();
        
        for (i, node) in self.nodes.iter().enumerate() {
            if node.point.dist(point) <= radius {
                neighbors.push(i);
            }
        }
        neighbors
    }

    // fn find_neighbors(&self, point: &Point) -> Vec<usize> {
    //     let mut neighbors = Vec::new();
    //     for (i, node) in self.nodes.iter().enumerate() {
    //         if node.point.dist(point) <= self.search_radius {
    //             neighbors.push(i);
    //         }
    //     }
    //     neighbors
    // }

    fn plan(&mut self) -> Option<Path> {
        for _ in 0..self.max_iter {
            let random_point = self.random_point();
            let nearest_idx = self.nearest_neighbor(&random_point);
            let new_point = self.steer(&self.nodes[nearest_idx].point, &random_point);
            
            if !self.collision_free(&self.nodes[nearest_idx].point, &new_point) {
                continue;
            }

            let neighbors = self.find_neighbors(&new_point);
            let mut min_cost = self.nodes[nearest_idx].cost + 
                self.nodes[nearest_idx].point.dist(&new_point);
            let mut min_parent = nearest_idx;

            // Choose best parent
            for &neighbor_idx in &neighbors {
                let potential_cost = self.nodes[neighbor_idx].cost + 
                    self.nodes[neighbor_idx].point.dist(&new_point);
                if potential_cost < min_cost && 
                    self.collision_free(&self.nodes[neighbor_idx].point, &new_point) {
                    min_cost = potential_cost;
                    min_parent = neighbor_idx;
                }
            }

            let new_node_idx = self.nodes.len();
            self.nodes.push(Node {
                point: new_point,
                parent: Some(min_parent),
                cost: min_cost,
            });

            // Rewire neighbors through new node if better
            for &neighbor_idx in &neighbors {
                let potential_cost = min_cost + new_point.dist(&self.nodes[neighbor_idx].point);
                if potential_cost < self.nodes[neighbor_idx].cost && 
                    self.collision_free(&new_point, &self.nodes[neighbor_idx].point) {
                    self.nodes[neighbor_idx].parent = Some(new_node_idx);
                    self.nodes[neighbor_idx].cost = potential_cost;
                    self.update_descendants_cost(neighbor_idx);
                }
            }

            let goal_dis: f64 = f64::EPSILON;
            if new_point.dist(&self.goal) < goal_dis && 
                self.collision_free(&new_point, &self.goal) {
                let final_node_idx = self.nodes.len();
                self.nodes.push(Node {
                    point: self.goal,
                    parent: Some(new_node_idx),
                    cost: min_cost + new_point.dist(&self.goal),
                });
                return Some(self.extract_path(final_node_idx));
            }
        }
        None
    }

    fn update_descendants_cost(&mut self, node_idx: usize) {
        // We need to track which nodes to update since we can't recursively
        // modify self.nodes
        let mut nodes_to_update = Vec::new();
        let node_point = self.nodes[node_idx].point;
        let node_cost = self.nodes[node_idx].cost;

        // Find all immediate children first
        for i in (node_idx + 1)..self.nodes.len() {
            if let Some(parent) = self.nodes[i].parent {
                if parent == node_idx {
                    let new_cost = node_cost + node_point.dist(&self.nodes[i].point);
                    nodes_to_update.push((i, new_cost));
                }
            }
        }

        // Update costs and continue looking for descendants
        while !nodes_to_update.is_empty() {
            let (current_idx, new_cost) = nodes_to_update.remove(0);
            self.nodes[current_idx].cost = new_cost;
            let current_point = self.nodes[current_idx].point;

            // Find children of this node
            for i in (current_idx + 1)..self.nodes.len() {
                if let Some(parent) = self.nodes[i].parent {
                    if parent == current_idx {
                        let child_new_cost = new_cost + current_point.dist(&self.nodes[i].point);
                        nodes_to_update.push((i, child_new_cost));
                    }
                }
            }
        }
    }

    fn extract_path(&self, goal_idx: usize) -> Path {
        let mut path_nodes = Vec::new();
        let mut current_idx = goal_idx;
        
        while let Some(parent_idx) = self.nodes[current_idx].parent {
            path_nodes.push(self.nodes[current_idx].point);
            current_idx = parent_idx;
        }
        path_nodes.push(self.start);
        path_nodes.reverse();
        
        Path { nodes: path_nodes }
    }
}

fn run_benchmark() -> (Vec<Path>, Duration, usize, usize) {
    let start = Point { x: 0.0, y: 0.0 };
    let goal = Point { x: 10.0, y: 10.0 };
    let obstacle = CircleObstacle {
        center: Point { x: 5.0, y: 5.0 },
        radius: 2.0,
    };
    
    let mut paths = Vec::new();
    let mut total_duration = Duration::new(0, 0);
    let mut successes = 0;
    let mut total_nodes = 0;
    
    for _ in 0..500 {
        let start_time = Instant::now();
        let mut rrt = RRTStar::new(
            start,
            goal,
            obstacle,
            10000,
            0.05,
            3.0,
            0.05,
            (Point { x: -2.0, y: -2.0 }, Point { x: 12.0, y: 12.0 }),
        );
        
        if let Some(path) = rrt.plan() {
            paths.push(path);
            successes += 1;
        }
        
        total_duration += start_time.elapsed();
        total_nodes += rrt.nodes.len();
    }
    
    (paths, total_duration, successes, total_nodes)
}

fn main() {
    let (paths, total_duration, successes, total_nodes) = run_benchmark();
    
    // Save paths to CSV file
    let mut file = File::create("paths.csv").unwrap();
    writeln!(file, "path_id,point_id,x,y").unwrap();
    
    for (path_idx, path) in paths.iter().enumerate() {
        for (point_idx, point) in path.nodes.iter().enumerate() {
            writeln!(file, "{},{},{},{}", path_idx, point_idx, point.x, point.y).unwrap();
        }
    }
    
    // Write benchmark results to text file
    let mut file = File::create("benchmark_results.txt").unwrap();
    writeln!(file, "Benchmark Results:").unwrap();
    writeln!(file, "Total runs: 500").unwrap();
    writeln!(file, "Successful paths: {}", successes).unwrap();
    writeln!(file, "Average time per run: {:?}", total_duration / 500).unwrap();
    writeln!(file, "Average memory (nodes) per run: {}", total_nodes / 500).unwrap();
}