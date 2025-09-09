/*
** Storage.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "types.h"
#include "Exception.h"
#include "Vec3.h"

#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <utility>
#include <algorithm>
#include <string>

namespace scone
{
	template< typename ValueT = Real, typename TimeT = TimeInSeconds >
	class Storage
	{
	public:
		class Frame
		{
		public:
			friend class Storage;

			Frame( Storage& store, TimeT t, ValueT default_value = ValueT( 0 ) ) :
				m_Store( store ),
				m_Time( t ),
				m_Values( store.GetChannelCount(), default_value ) { }

			TimeT GetTime() const { return m_Time; }

			ValueT& operator[]( index_t idx ) { return m_Values[idx]; }

			const ValueT& operator[]( index_t idx ) const { return m_Values[idx]; }

			ValueT& operator[]( const String& label ) {
				index_t idx = m_Store.TryGetChannelIndex( label );
				if ( idx == NoIndex )
					idx = m_Store.AddChannel( label );
				return m_Values[idx];
			}

			const ValueT& operator[]( const String& label ) const {
				index_t idx = m_Store.GetChannelIndex( label );
				SCONE_ASSERT( idx != NoIndex );
				return m_Values[idx];
			}

			const std::vector< ValueT >& GetValues() const { return m_Values; }

			void Set( const String& label, Real v ) { ( *this )[label] = v; }

			void SetVec3( const String& label, const Vec3& vec ) {
				( *this )[label + "_x"] = vec.x;
				( *this )[label + "_y"] = vec.y;
				( *this )[label + "_z"] = vec.z;
			}

			Vec3 GetVec3( index_t idx ) const {
				SCONE_ASSERT( idx != NoIndex && idx + 2 < m_Values.size() );
				return Vec3( m_Values[idx], m_Values[idx + 1], m_Values[idx + 2] );
			}

			Vec3 GetVec3( const String& label ) const {
				return GetVec3( m_Store.TryGetChannelIndex( label + "_x" ) );
			}

		private:
			Storage< ValueT, TimeT >& m_Store;
			TimeT m_Time;
			std::vector< ValueT > m_Values;
		};

		using FrameUP = std::unique_ptr< Frame >;
		using container_t = std::vector< FrameUP >;

		Storage() {}
		Storage( const Storage& other ) {
			*this = other;
		};
		Storage( Storage&& other ) {
			*this = std::move( other );
		};
		Storage( const std::vector< String >& labels ) : m_Labels( labels ) {
			for ( index_t i = 0; i < m_Labels.size(); ++i )
				m_LabelIndexMap[m_Labels[i]] = i;
		}
		Storage& operator=( const Storage& other ) {
			m_Labels = other.m_Labels;
			m_LabelIndexMap = other.m_LabelIndexMap;
			m_Data.clear();
			m_Data.reserve( other.m_Data.size() );
			for ( auto it = other.m_Data.begin(); it != other.m_Data.end(); ++it )
				m_Data.push_back( std::make_unique<Frame>( **it ) );
			m_InterpolationCache.clear();
			return *this;
		};
		Storage& operator=( Storage&& other ) {
			m_Labels = std::move( other.m_Labels );
			m_LabelIndexMap = std::move( other.m_LabelIndexMap );
			m_Data = std::move( other.m_Data );
			m_InterpolationCache.clear();
			return *this;
		};

		// clear everything
		void Clear() { m_Labels.clear(); m_LabelIndexMap.clear(); m_Data.clear(); m_InterpolationCache.clear(); }

		// erase frames, keep columns
		void EraseFrames( index_t start, size_t size = no_size ) { 
			if ( start < m_Data.size() ) {
				auto begin_it = m_Data.begin() + start;
				auto end_it = ( size == no_size || start + size >= m_Data.size() ) ? m_Data.end() : begin_it + size;
				m_Data.erase( begin_it, end_it );
				m_InterpolationCache.clear();
			}
		}

		Storage CopySlice( size_t start, size_t size, size_t stride ) const {
			SCONE_ASSERT( stride > 0 );
			Storage r;
			r.m_Labels = m_Labels;
			r.m_LabelIndexMap = m_LabelIndexMap;
			if ( size == 0 || size > GetFrameCount() / stride )
				size = ( GetFrameCount() + stride - 1 ) / stride;
			r.m_Data.reserve( size );
			for ( size_t i = start; r.m_Data.size() < size && i < m_Data.size(); i += stride )
				r.m_Data.push_back( std::make_unique<Frame>( *m_Data[i] ) );
			return r;
		}

		Frame& AddFrame( TimeT time, ValueT default_value = ValueT( 0 ) ) {
			SCONE_ERROR_IF( !m_Data.empty() && time <= m_Data.back()->GetTime(),
				"Timestamp is not higher than previous frame time: " + std::to_string( time ) );
			m_Data.push_back( std::make_unique<Frame>( *this, time, default_value ) );
			m_InterpolationCache.clear(); // cached iterators have become invalid
			return *m_Data.back();
		}

		bool IsEmpty() const { return m_Data.empty(); }

		Frame& Front() { SCONE_ASSERT( !m_Data.empty() ); return *m_Data.front(); }
		const Frame& Front() const { SCONE_ASSERT( !m_Data.empty() ); return *m_Data.front(); }

		Frame& Back() { SCONE_ASSERT( !m_Data.empty() ); return *m_Data.back(); }
		const Frame& Back() const { SCONE_ASSERT( !m_Data.empty() ); return *m_Data.back(); }

		Frame& GetFrame( index_t frame_idx ) { SCONE_ASSERT( frame_idx < m_Data.size() ); return *m_Data[frame_idx]; }
		const Frame& GetFrame( index_t frame_idx ) const { SCONE_ASSERT( frame_idx < m_Data.size() ); return *m_Data[frame_idx]; }

		std::vector< ValueT > GetChannelData( index_t idx ) const {
			std::vector< ValueT > result( GetFrameCount() );
			for ( index_t f = 0; f < GetFrameCount(); ++f )
				result[f] = GetFrame( f )[idx];
			return result;
		}

		std::vector< TimeT > GetTimeData() const {
			std::vector< TimeT > result( GetFrameCount() );
			for ( index_t f = 0; f < GetFrameCount(); ++f )
				result[f] = GetFrame( f ).GetTime();
			return result;
		}

		size_t GetFrameCount() const { return m_Data.size(); }

		Real GetAverageFrameDuration() const { 
			if ( m_Data.size() >= 2 )
				return ( m_Data.back()->GetTime() - m_Data.front()->GetTime() ) / ( m_Data.size() - 1 );
			else return 0.0;
		}

		index_t AddChannel( const String& label, ValueT default_value = ValueT( 0 ) ) {
			SCONE_ASSERT_MSG( TryGetChannelIndex( label ) == NoIndex, "Channel " + label + " already exists" );
			m_Labels.push_back( label );
			m_LabelIndexMap[label] = m_Labels.size() - 1;
			for ( auto it = m_Data.begin(); it != m_Data.end(); ++it )
				( *it )->m_Values.resize( m_Labels.size(), default_value ); // resize existing data
			return m_Labels.size() - 1;
		}

		index_t GetChannelIndex( const String& label ) const {
			auto it = m_LabelIndexMap.find( label );
			SCONE_THROW_IF( it == m_LabelIndexMap.end(), "Could not find channel " + label );
			return it->second;
		}

		index_t TryGetChannelIndex( const String& label ) const {
			auto it = m_LabelIndexMap.find( label );
			if ( it == m_LabelIndexMap.end() )
				return NoIndex;
			else return it->second;
		}

		size_t GetChannelCount() const { return m_Labels.size(); }
		const std::vector< String >& GetLabels() const { return m_Labels; }
		const String& GetLabel( index_t idx ) const { return m_Labels[idx]; }
		const container_t& GetData() const { return m_Data; }

		index_t GetClosestFrameIndex( TimeT time ) const {
			SCONE_ASSERT( !m_Data.empty() );
			auto it = upper_bound( time );
			if ( it == m_Data.begin() )
				return it - m_Data.begin();
			else if ( it == m_Data.end() )
				return m_Data.size() - 1;
			else {
				auto it0 = it;
				--it0;
				if ( time - ( *it0 )->GetTime() <= ( *it )->GetTime() - time )
					return it0 - m_Data.begin();
				else return it - m_Data.begin();
			}
		}

		const Frame& GetClosestFrame( TimeT time ) const { return GetFrame( GetClosestFrameIndex( time ) ); }

		// interpolation related stuff
		struct InterpolatedFrame {
			double upper_weight;
			typename container_t::const_iterator upper_frame, lower_frame;
			ValueT value( index_t channel_idx ) const { return upper_weight * ( **upper_frame )[channel_idx] + ( 1.0 - upper_weight ) * ( **lower_frame )[channel_idx]; }
		};

		// Compute interpolated value (always recomputes, slow)
		ValueT ComputeInterpolatedValue( TimeT time, index_t idx ) const {
			SCONE_ASSERT( !m_Data.empty() );
			return ComputeInterpolatedFrame( time ).value( idx );
		}

		// Compute interpolated frame (always recomputes)
		InterpolatedFrame ComputeInterpolatedFrame( TimeT time ) const {
			InterpolatedFrame bf;
			bf.upper_frame = upper_bound( time );
			if ( bf.upper_frame == m_Data.cend() )
			{
				// timestamp too high, point to most recent frame
				bf.lower_frame = bf.upper_frame = m_Data.begin() + m_Data.size() - 1;
				bf.upper_weight = 1.0;
			}
			else if ( bf.upper_frame == m_Data.cbegin() )
			{
				// timestamp too low, point to oldest frame
				bf.lower_frame = bf.upper_frame;
				bf.upper_weight = 1.0;
			}
			else
			{
				// we have an actual interpolation
				bf.lower_frame = bf.upper_frame - 1;
				bf.upper_weight = ( time - ( *bf.lower_frame )->GetTime() ) / ( ( *bf.upper_frame )->GetTime() - ( *bf.lower_frame )->GetTime() );
			}

			return bf;
		}

		// Get interpolated value, check cached results first
		ValueT GetInterpolatedValue( TimeT time, index_t idx ) {
			SCONE_ASSERT( !m_Data.empty() );
			return GetInterpolatedFrame( time ).value( idx );
		}

		// Get interpolated frame, check cached results first
		InterpolatedFrame GetInterpolatedFrame( TimeT time ) {
			auto cacheIt = m_InterpolationCache.find( time );
			if ( cacheIt != m_InterpolationCache.end() )
				return cacheIt->second;

			InterpolatedFrame bf = ComputeInterpolatedFrame( time );
			m_InterpolationCache[time] = bf;

			return bf;
		}

	private:
		std::vector< String > m_Labels;
		container_t m_Data;
		std::unordered_map< String, index_t > m_LabelIndexMap;

		auto upper_bound( TimeT time ) const {
			return std::upper_bound( m_Data.cbegin(), m_Data.cend(), time, []( TimeT lhs, const FrameUP& rhs ) { return lhs < rhs->GetTime(); } );
		}

		std::map< TimeT, InterpolatedFrame > m_InterpolationCache;
	};
}
