# Represents a set of cone associations

# Metadata relating to association type
string type  # Association type
float64 threshold

# Information relating to matched cones
ConeAssociation[] matched

# Information relating to unmatched cones
ConeWithCovariance[] unmatched
