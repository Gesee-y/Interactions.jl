######################################################################################################################
################################################### COLLISION GRID ###################################################
######################################################################################################################

export CollisionGrid, ProcessObjects

######################################################## CORE ########################################################

const BIT_WIDTH = 5 # The number of bits allocated for a given data
const DEFAULT_MASK = (UInt64(1) << BIT_WIDTH) - 1 # The default mask to get a particular data

struct CollisionGrid{T,N,L}
	mask::UInt # The lookup mask for the bitboard
	size::NTuple{2,Int} # The size of the world to skip offscreen entities
	zones::Matrix{Vector{T}} # Our Vector of zones
	bitboard::MagicBitboard # The bitboard (see `Arceus.jl`)

	# Constructor

	function CollisionGrid{T,N}(size) where {T <: Any, N}
		mask = UInt64((1 << (2BIT_WIDTH))-1) # We get our look up mask, a serie of 1
		zones = Matrix{Vector{T}}(undef, N, N)
		for i in eachindex(zones)
			zones[i] = T[]
		end

		new{T,N,N*N}(mask, (size[1], size[2]), zones, MagicBitboard(mask,0,44, MakeRoute, Vector{UInt64}; PEXT=true))
	end
end

##################################################### FUNCTIONS ######################################################

"""
    ProcessObjects(grid::CollisionGrid, obj::Vector)

Resolve collision for a given set of entities `obj`.
This will first collect the plausible collisions using a 2D grib based on bitboards for fast repartitions.
"""
function ProcessObjects(grid::CollisionGrid, obj::Vector)
	empty!.(grid.zones)
	for o in obj
		SetInZone(grid, o)
	end

	zones = filter(v -> length(v) > 1, grid.zones)
	for zone in zones
		# The collision resolution
	end
end

"""
    SetInZone(grid::CollisionGrid{N,L}, obj::AbstractBody)

This automatically route the object `obj` to the correct grids cells using bitboard lookup, making this function
super fast.
"""
function SetInZone(grid::CollisionGrid{T,N,L}, obj::T) where {T,N,L}
	x0,y0,w0,h0 = get_dimensions(obj)
	size = 1 << BIT_WIDTH # The size of our mask

	x, w = _map_to_range(x0,1:size,grid.size[1]), _map_to_range(w0,1:size,grid.size[1])
	y, h = _map_to_range(y0,1:size,grid.size[2]), _map_to_range(h0,1:size,grid.size[2])

    !(0 ≤ x ≤ 31 && 0 ≤ y ≤ 31) && return

	traits = to_bit_boundingbox(w,h,1,1)
	zones::Vector{UInt64} = grid.bitboard[traits]
    
    data = grid.zones
    offset = _compute_coordinate((x,y), (size,size))
	@inbounds for i = x:x+w, j=y:y+h
		idx = i + (j-1)*size + offset
	#@inbounds for i in zones
	#	idx = i+offset
		idx <= L && push!(data[idx], obj)
	end
end

# Create the bounding box of the object but as a collection of bits
to_bit_boundingbox(w, h, x, y;size=BIT_WIDTH)::UInt = (y << size*3) | (x << size*2) | (h << size) | (w)

"""
    MakeRoute(x::UInt64)

This is our lookup function, it takes a bit bounding box as input and output all the zones he is in.
Using magic bitboard, foreach possible bounding box, a collection of zones are directly mapped to it
"""
function MakeRoute(x::UInt64)
	# We collect our data just by decoding them from `x`.
	# I guess it's not so hard to understand.
	# We first use the mask to put to zero all irrelevants data with a AND
	# Then if necessary we use shift to put the data at their correct position, 
	# this remove the offset we added to them in `to_bit_boundingbox`
	sz = BIT_WIDTH
	w, h = (x & DEFAULT_MASK) + 1, ((x & (DEFAULT_MASK << sz)) >> sz) + 1
	x_start, y_start = ((x & (DEFAULT_MASK << sz*2)) >> sz*2) + 1, ((x & (DEFAULT_MASK << sz*3)) >> sz*3) + 1

    result::Vector{UInt64} = UInt64[]
    size = 1 << sz # Faster than doing 2^sz
	
	for i in x_start:w
		for j in y_start:h
			push!(result, _compute_coordinate((i,j),(size,size)))
		end
	end

	return result
end

function _map_to_range(x,r,M)
	t = x/M
	v = Int(floor(t*r[end]))

	return v
end

_compute_coordinate((i, j), (width, height)) = (j - 1) * width + i
